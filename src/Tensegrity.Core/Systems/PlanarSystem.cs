using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Systems;

/// <summary>
/// A set of planar rigid links under uniform gravity, joined by pin
/// constraints and cables, in maximal coordinates.
///
/// State layout for n bodies (length 6n + 2), positions first, then
/// velocities, then the energy dissipated so far and the work done by the
/// joint actuators so far:
/// <code>
/// [ x0 y0 θ0  x1 y1 θ1 … | ẋ0 ẏ0 θ̇0  ẋ1 ẏ1 θ̇1 … | E_diss  W_act ]
/// </code>
/// Splitting positions from velocities keeps the generalized-coordinate block
/// contiguous for the constraint solver. E_diss and W_act are integrated with
/// the rest of the state, so the energy balance (PLAN.md 4.3)
/// <code>
/// kinetic + gravitational + elastic + dissipated − actuator work = constant
/// </code>
/// holds to integrator accuracy; see <see cref="TotalEnergy"/>.
///
/// Pins are enforced with Lagrange multipliers (PLAN.md 4.2, 9.1). At every
/// derivative evaluation the constraint forces are solved from
/// <code>
/// (J·M⁻¹·Jᵀ)·λ = −J·M⁻¹·F − J̇·q̇ − 2ω·Ċ − ω²·C
/// </code>
/// with C the pin position error, J = ∂C/∂q, M the diagonal mass matrix,
/// F the applied forces and ω the <see cref="ConstraintStabilization"/>
/// frequency; then q̈ = M⁻¹·(F + Jᵀ·λ).
///
/// Cables switch between slack, taut and recoiling (see <see cref="Cable"/>),
/// and actuator schedules between their linear segments (see
/// <see cref="TorqueSchedule"/>). As an <see cref="ISwitchedSystem"/>,
/// <see cref="Evaluate"/> uses each cable's and schedule's latched mode, so a
/// system with cables or non-constant schedules must be stepped by an
/// <see cref="EventSplittingStepper"/> (or have its modes latched by hand).
/// Diagnostics such as <see cref="GetCableTensions"/>,
/// <see cref="GetActuatorTorques"/> and <see cref="GetPinReactions"/> always
/// use the physical laws at the given time and state, independent of the
/// latch.
///
/// Instances keep scratch buffers between calls, so one instance must not be
/// evaluated from several threads at once.
/// </summary>
public sealed class PlanarSystem : ISwitchedSystem
{
    /// <summary>Standard gravity used throughout the study (PLAN.md 4.1), m/s².</summary>
    public static readonly Vector2d StandardGravity = new(0, -9.81);

    public const int DofPerBody = 3;

    private const int Ground = -1;

    private readonly PlanarLink[] links;
    private readonly PinJoint[] pins;
    private readonly ResolvedPoint[] pinA;
    private readonly ResolvedPoint[] pinB;
    private readonly Cable[] cables;
    private readonly ResolvedPoint[] cableA;
    private readonly ResolvedPoint[] cableB;
    private readonly CableMode[] latchedModes;
    private readonly JointActuator[] actuators;
    private readonly int[] actuatorPin;
    private readonly int[] latchedSegments;
    private bool modesLatched;

    // Scratch for the constraint solve, sized once.
    private readonly double[] forces;
    private readonly double[] jacobian;
    private readonly double[] constraintMatrix;
    private readonly double[] multipliers;

    public PlanarSystem(
        IEnumerable<PlanarLink> links,
        Vector2d gravity,
        IEnumerable<PinJoint>? pins = null,
        IEnumerable<Cable>? cables = null,
        ConstraintStabilization? stabilization = null,
        IEnumerable<JointActuator>? actuators = null)
    {
        ArgumentNullException.ThrowIfNull(links);
        this.links = links.ToArray();
        if (this.links.Length == 0)
        {
            throw new ArgumentException("A system needs at least one link.", nameof(links));
        }

        RequireUniqueIds(this.links.Select(l => l.Id), "link", nameof(links));

        if (!double.IsFinite(gravity.X) || !double.IsFinite(gravity.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(gravity), gravity, "Must be finite.");
        }

        Gravity = gravity;
        Stabilization = stabilization ?? ConstraintStabilization.Default;

        this.pins = pins?.ToArray() ?? [];
        RequireUniqueIds(this.pins.Select(p => p.Id), "pin", nameof(pins));
        pinA = this.pins.Select(p => Resolve(p.A, $"pin '{p.Id}'")).ToArray();
        pinB = this.pins.Select(p => Resolve(p.B, $"pin '{p.Id}'")).ToArray();

        this.cables = cables?.ToArray() ?? [];
        RequireUniqueIds(this.cables.Select(c => c.Id), "cable", nameof(cables));
        cableA = this.cables.Select(c => Resolve(c.A, $"cable '{c.Id}'")).ToArray();
        cableB = this.cables.Select(c => Resolve(c.B, $"cable '{c.Id}'")).ToArray();
        latchedModes = new CableMode[this.cables.Length];

        this.actuators = actuators?.ToArray() ?? [];
        RequireUniqueIds(this.actuators.Select(a => a.Id), "actuator", nameof(actuators));
        actuatorPin = this.actuators.Select(ResolvePin).ToArray();
        var doubled = actuatorPin.GroupBy(p => p).FirstOrDefault(g => g.Count() > 1);
        if (doubled is not null)
        {
            throw new ArgumentException($"Pin '{this.pins[doubled.Key].Id}' has more than one actuator.", nameof(actuators));
        }

        latchedSegments = new int[this.actuators.Length];

        // With nothing that switches, the default latch is already correct.
        modesLatched = this.cables.Length == 0 && this.actuators.All(a => a.Schedule.IsConstant);

        forces = new double[CoordinateCount];
        jacobian = new double[ConstraintCount * CoordinateCount];
        constraintMatrix = new double[ConstraintCount * ConstraintCount];
        multipliers = new double[ConstraintCount];
    }

    public IReadOnlyList<PlanarLink> Links => links;

    public IReadOnlyList<PinJoint> Pins => pins;

    public IReadOnlyList<Cable> Cables => cables;

    public IReadOnlyList<JointActuator> Actuators => actuators;

    public Vector2d Gravity { get; }

    public ConstraintStabilization Stabilization { get; }

    /// <summary>Number of scalar constraint equations: 2 per pin.</summary>
    public int ConstraintCount => 2 * pins.Length;

    public int BodyCount => links.Length;

    /// <summary>Number of generalized coordinates: 3 per body.</summary>
    public int CoordinateCount => DofPerBody * links.Length;

    /// <summary>Length of the state vector: coordinates, velocities, dissipated energy and actuator work.</summary>
    public int StateLength => (2 * CoordinateCount) + 2;

    /// <summary>Index of the cumulative dissipated energy in the state vector.</summary>
    public int DissipatedEnergyIndex => 2 * CoordinateCount;

    /// <summary>Index of the cumulative actuator work in the state vector.</summary>
    public int ActuatorWorkIndex => (2 * CoordinateCount) + 1;

    public int PositionIndex(int body) => DofPerBody * CheckBody(body);

    public int VelocityIndex(int body) => CoordinateCount + (DofPerBody * CheckBody(body));

    public SystemState CreateState(double time = 0) => new(StateLength, time);

    public Pose2d GetPose(ReadOnlySpan<double> state, int body)
    {
        CheckLength(state.Length);
        var i = PositionIndex(body);
        return new Pose2d(new Vector2d(state[i], state[i + 1]), state[i + 2]);
    }

    public Twist2d GetTwist(ReadOnlySpan<double> state, int body)
    {
        CheckLength(state.Length);
        var i = VelocityIndex(body);
        return new Twist2d(new Vector2d(state[i], state[i + 1]), state[i + 2]);
    }

    public void SetBody(Span<double> state, int body, Pose2d pose, Twist2d twist)
    {
        CheckLength(state.Length);
        var p = PositionIndex(body);
        state[p] = pose.Position.X;
        state[p + 1] = pose.Position.Y;
        state[p + 2] = pose.Angle;
        var v = VelocityIndex(body);
        state[v] = twist.Linear.X;
        state[v + 1] = twist.Linear.Y;
        state[v + 2] = twist.Angular;
    }

    /// <summary>
    /// dy/dt for the whole system. Applied forces and torques are accumulated
    /// per body, pin constraint forces are added, and the sum is divided by
    /// mass and inertia, so later force elements slot in without changing
    /// the structure.
    /// </summary>
    public void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative)
    {
        CheckLength(state.Length);
        CheckLength(derivative.Length);
        var n = CoordinateCount;

        // dq/dt = v
        state.Slice(n, n).CopyTo(derivative[..n]);

        if (!modesLatched)
        {
            throw new InvalidOperationException(
                "Modes are not latched. Step a system with cables or scheduled actuators through an EventSplittingStepper, or call LatchModes first.");
        }

        ComputeTotalForces(time, state, useLatchedModes: true);

        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var i = DofPerBody * b;
            derivative[n + i] = forces[i] / link.Mass;
            derivative[n + i + 1] = forces[i + 1] / link.Mass;
            derivative[n + i + 2] = forces[i + 2] / link.MomentOfInertia;
        }

        derivative[DissipatedEnergyIndex] = DissipatedPower(state);
        derivative[ActuatorWorkIndex] = ActuatorPower(time, state, useLatchedModes: true);
    }

    /// <summary>
    /// The force each pin exerts on its end A at this time and state, N. End
    /// B receives the opposite force. Includes the stabilization terms, which
    /// vanish on the constraint manifold.
    /// </summary>
    public void GetPinReactions(double time, ReadOnlySpan<double> state, Span<Vector2d> reactions)
    {
        CheckLength(state.Length);
        if (reactions.Length != pins.Length)
        {
            throw new ArgumentException($"Expected {pins.Length} reactions, got {reactions.Length}.", nameof(reactions));
        }

        ComputeTotalForces(time, state, useLatchedModes: false);
        for (var p = 0; p < pins.Length; p++)
        {
            reactions[p] = new Vector2d(multipliers[2 * p], multipliers[(2 * p) + 1]);
        }
    }

    /// <summary>
    /// Axial force across each link's midpoint section at this state,
    /// compression positive, N: the force the link's −x half exerts on its
    /// +x half, along the link axis. From Newton's law for the +x half,
    /// <code>
    /// F_section = (m/2)·a_q − F_applied(+x half) − (m/2)·g
    /// </code>
    /// where a_q is the acceleration of the half's center (the quarter
    /// point), so the link's own weight and inertia are included. Assumes a
    /// uniform bar, so the +x half has mass m/2 centered at local (ℓ/4, 0).
    /// Cable and pin forces count toward the +x half when their attachment
    /// has local x &gt; 0; a point exactly on the midpoint acts on neither.
    /// Uses the physical cable law and actuator torques at the given time
    /// and state. Actuator torques are pure couples, so they enter only
    /// through the link's acceleration.
    /// </summary>
    public void GetMidpointCompressions(double time, ReadOnlySpan<double> state, Span<double> compressions)
    {
        CheckLength(state.Length);
        if (compressions.Length != links.Length)
        {
            throw new ArgumentException($"Expected {links.Length} values, got {compressions.Length}.", nameof(compressions));
        }

        ComputeTotalForces(time, state, useLatchedModes: false);

        Span<Vector2d> halfForce = links.Length <= 64 ? stackalloc Vector2d[links.Length] : new Vector2d[links.Length];
        halfForce.Clear();

        void AddToHalf(Span<Vector2d> half, ResolvedPoint point, Vector2d force)
        {
            if (point.Body != Ground && point.Local.X > 0)
            {
                half[point.Body] += force;
            }
        }

        for (var c = 0; c < cables.Length; c++)
        {
            var (length, rate, direction) = CableGeometry(state, c);
            var force = cables[c].Tension(length, rate) * direction;
            AddToHalf(halfForce, cableA[c], force);
            AddToHalf(halfForce, cableB[c], -force);
        }

        for (var p = 0; p < pins.Length; p++)
        {
            var force = new Vector2d(multipliers[2 * p], multipliers[(2 * p) + 1]);
            AddToHalf(halfForce, pinA[p], force);
            AddToHalf(halfForce, pinB[p], -force);
        }

        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var i = DofPerBody * b;
            var linear = new Vector2d(forces[i], forces[i + 1]) / link.Mass;
            var angular = forces[i + 2] / link.MomentOfInertia;
            var pose = GetPose(state, b);
            var omega = GetTwist(state, b).Angular;
            var axis = pose.RotateToWorld(Vector2d.UnitX);
            var quarter = (link.Length / 4) * axis;
            var quarterAcceleration = linear + Vector2d.Cross(angular, quarter) - (omega * omega * quarter);
            var halfMass = link.Mass / 2;
            var section = (halfMass * quarterAcceleration) - halfForce[b] - (halfMass * Gravity);
            compressions[b] = Vector2d.Dot(section, axis);
        }
    }

    /// <summary>Largest pin position error |C| over all pins, m. Zero with no pins.</summary>
    public double PinPositionResidual(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var worst = 0.0;
        for (var p = 0; p < pins.Length; p++)
        {
            worst = Math.Max(worst, (WorldPoint(state, pinA[p]) - WorldPoint(state, pinB[p])).Length);
        }

        return worst;
    }

    /// <summary>Largest pin velocity error |Ċ| over all pins, m/s. Zero with no pins.</summary>
    public double PinVelocityResidual(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var worst = 0.0;
        for (var p = 0; p < pins.Length; p++)
        {
            worst = Math.Max(worst, (PointVelocity(state, pinA[p]) - PointVelocity(state, pinB[p])).Length);
        }

        return worst;
    }

    /// <inheritdoc/>
    public void LatchModes(double time, ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        for (var c = 0; c < cables.Length; c++)
        {
            latchedModes[c] = CableModeAt(state, c);
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            latchedSegments[a] = actuators[a].Schedule.SegmentAt(time);
        }

        modesLatched = true;
    }

    /// <inheritdoc/>
    public bool ModesMatch(double time, ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        if (!modesLatched)
        {
            return false;
        }

        for (var c = 0; c < cables.Length; c++)
        {
            if (CableModeAt(state, c) != latchedModes[c])
            {
                return false;
            }
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            if (actuators[a].Schedule.SegmentAt(time) != latchedSegments[a])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Length L and rate L̇ of cable <paramref name="cable"/> at this state.</summary>
    public (double Length, double Rate) GetCableLength(ReadOnlySpan<double> state, int cable)
    {
        CheckLength(state.Length);
        if ((uint)cable >= (uint)cables.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(cable), cable, $"System has {cables.Length} cables.");
        }

        var (length, rate, _) = CableGeometry(state, cable);
        return (length, rate);
    }

    /// <summary>The physical mode of cable <paramref name="cable"/> at this state.</summary>
    public CableMode GetCableMode(ReadOnlySpan<double> state, int cable)
    {
        var (length, rate) = GetCableLength(state, cable);
        return cables[cable].ModeAt(length, rate);
    }

    /// <summary>Physical tension of every cable at this state, N.</summary>
    public void GetCableTensions(ReadOnlySpan<double> state, Span<double> tensions)
    {
        CheckLength(state.Length);
        if (tensions.Length != cables.Length)
        {
            throw new ArgumentException($"Expected {cables.Length} tensions, got {tensions.Length}.", nameof(tensions));
        }

        for (var c = 0; c < cables.Length; c++)
        {
            var (length, rate, _) = CableGeometry(state, c);
            tensions[c] = cables[c].Tension(length, rate);
        }
    }

    /// <summary>Torque of every actuator at <paramref name="time"/>, N·m, from its schedule.</summary>
    public void GetActuatorTorques(double time, Span<double> torques)
    {
        if (torques.Length != actuators.Length)
        {
            throw new ArgumentException($"Expected {actuators.Length} torques, got {torques.Length}.", nameof(torques));
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            torques[a] = actuators[a].Schedule.TorqueAt(time);
        }
    }

    /// <summary>World position of an attachment at this state.</summary>
    public Vector2d WorldPoint(ReadOnlySpan<double> state, Attachment attachment) =>
        WorldPoint(state, Resolve(attachment, "attachment"));

    /// <summary>World velocity of an attachment at this state.</summary>
    public Vector2d PointVelocity(ReadOnlySpan<double> state, Attachment attachment) =>
        PointVelocity(state, Resolve(attachment, "attachment"));

    /// <summary>Translational plus rotational kinetic energy, J.</summary>
    public double KineticEnergy(ReadOnlySpan<double> state)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var twist = GetTwist(state, b);
            total += (0.5 * link.Mass * twist.Linear.LengthSquared)
                + (0.5 * link.MomentOfInertia * twist.Angular * twist.Angular);
        }

        return total;
    }

    /// <summary>Gravitational potential energy, −Σ m·(g·r), zero at the origin, J.</summary>
    public double GravitationalPotentialEnergy(ReadOnlySpan<double> state)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            total -= links[b].Mass * Vector2d.Dot(Gravity, GetPose(state, b).Position);
        }

        return total;
    }

    /// <summary>Elastic energy stored in the cables, Σ ½·k·(L − L0)² over stretched cables, J.</summary>
    public double CableElasticEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var total = 0.0;
        for (var c = 0; c < cables.Length; c++)
        {
            total += ElasticEnergy(cables[c], CableGeometry(state, c).Length);
        }

        return total;
    }

    /// <summary>Energy dissipated by cable damping since the start of the run, J.</summary>
    public double DissipatedEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[DissipatedEnergyIndex];
    }

    /// <summary>Work done on the system by the joint actuators since the start of the run, J.</summary>
    public double ActuatorWork(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[ActuatorWorkIndex];
    }

    /// <summary>Kinetic + gravitational + cable elastic energy, J.</summary>
    public double MechanicalEnergy(ReadOnlySpan<double> state) =>
        KineticEnergy(state) + GravitationalPotentialEnergy(state) + CableElasticEnergy(state);

    /// <summary>
    /// Mechanical energy plus energy dissipated so far, minus the work the
    /// actuators have put in, J. It is conserved, so its drift measures the
    /// integration error (PLAN.md 4.3, test 7).
    /// </summary>
    public double TotalEnergy(ReadOnlySpan<double> state) =>
        MechanicalEnergy(state) + DissipatedEnergy(state) - ActuatorWork(state);

    /// <summary>Total angular momentum about a fixed world point, kg·m²/s.</summary>
    public double AngularMomentum(ReadOnlySpan<double> state, Vector2d about)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var twist = GetTwist(state, b);
            var r = GetPose(state, b).Position - about;
            total += (link.MomentOfInertia * twist.Angular) + (link.Mass * Vector2d.Cross(r, twist.Linear));
        }

        return total;
    }

    /// <summary>
    /// Fills <see cref="forces"/> with the applied generalized forces plus
    /// the pin constraint forces Jᵀ·λ, and <see cref="multipliers"/> with λ.
    /// </summary>
    private void ComputeTotalForces(double time, ReadOnlySpan<double> state, bool useLatchedModes)
    {
        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var i = DofPerBody * b;

            // Uniform gravity acts at the center of mass: no torque.
            forces[i] = link.Mass * Gravity.X;
            forces[i + 1] = link.Mass * Gravity.Y;
            forces[i + 2] = 0.0;
        }

        for (var c = 0; c < cables.Length; c++)
        {
            var cable = cables[c];
            var (length, rate, direction) = CableGeometry(state, c);
            var mode = useLatchedModes ? latchedModes[c] : cable.ModeAt(length, rate);
            var tension = cable.TensionInMode(mode, length, rate);

            // The cable pulls end A toward B and end B toward A.
            AddPointForce(state, cableA[c], tension * direction);
            AddPointForce(state, cableB[c], -tension * direction);
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            var torque = ActuatorTorque(a, time, useLatchedModes);
            var p = actuatorPin[a];
            AddTorque(pinA[p], torque);
            AddTorque(pinB[p], -torque);
        }

        if (pins.Length > 0)
        {
            AddConstraintForces(state);
        }
    }

    private void AddConstraintForces(ReadOnlySpan<double> state)
    {
        var rows = ConstraintCount;
        var cols = CoordinateCount;
        var omega = Stabilization.Frequency;
        var j = jacobian.AsSpan();
        var a = constraintMatrix.AsSpan();
        var rhs = multipliers.AsSpan();
        j.Clear();

        // Jacobian rows and the right-hand side −J̇·q̇ − 2ω·Ċ − ω²·C.
        for (var p = 0; p < pins.Length; p++)
        {
            var c = WorldPoint(state, pinA[p]) - WorldPoint(state, pinB[p]);
            var cDot = PointVelocity(state, pinA[p]) - PointVelocity(state, pinB[p]);
            var gamma = AddJacobianBlock(state, j, p, pinA[p], +1.0) + AddJacobianBlock(state, j, p, pinB[p], -1.0);
            var bias = gamma + (2 * omega * cDot) + (omega * omega * c);
            rhs[2 * p] = -bias.X;
            rhs[(2 * p) + 1] = -bias.Y;
        }

        // A = J·M⁻¹·Jᵀ and rhs −= J·M⁻¹·F. Only the lower triangle of A is used.
        for (var r = 0; r < rows; r++)
        {
            var jr = j.Slice(r * cols, cols);
            var jwf = 0.0;
            for (var k = 0; k < cols; k++)
            {
                jwf += jr[k] * InverseMass(k) * forces[k];
            }

            rhs[r] -= jwf;

            for (var s = 0; s <= r; s++)
            {
                var js = j.Slice(s * cols, cols);
                var sum = 0.0;
                for (var k = 0; k < cols; k++)
                {
                    sum += jr[k] * InverseMass(k) * js[k];
                }

                a[(r * rows) + s] = sum;
            }
        }

        if (!DenseCholesky.TryFactor(a, rows))
        {
            throw new InvalidOperationException(
                "Pin constraint matrix is singular: the pins are redundant or degenerate at this configuration.");
        }

        DenseCholesky.Solve(a, rows, rhs);

        // F += Jᵀ·λ
        for (var r = 0; r < rows; r++)
        {
            var jr = j.Slice(r * cols, cols);
            var lambda = rhs[r];
            for (var k = 0; k < cols; k++)
            {
                forces[k] += jr[k] * lambda;
            }
        }
    }

    /// <summary>
    /// Writes sign·∂(point)/∂q into the two Jacobian rows of pin
    /// <paramref name="pin"/> and returns sign·J̇·q̇ for the point, which is
    /// the centripetal term −θ̇²·r. Ground points contribute nothing.
    /// </summary>
    private Vector2d AddJacobianBlock(ReadOnlySpan<double> state, Span<double> j, int pin, ResolvedPoint point, double sign)
    {
        if (point.Body == Ground)
        {
            return Vector2d.Zero;
        }

        var cols = CoordinateCount;
        var pose = GetPose(state, point.Body);
        var r = pose.RotateToWorld(point.Local);
        var c = DofPerBody * point.Body;
        var rowX = 2 * pin * cols;
        var rowY = rowX + cols;

        // d(x + R(θ)·ℓ)/d(x, y, θ) = [[1, 0, −r_y], [0, 1, r_x]].
        j[rowX + c] += sign;
        j[rowX + c + 2] += sign * -r.Y;
        j[rowY + c + 1] += sign;
        j[rowY + c + 2] += sign * r.X;

        var w = GetTwist(state, point.Body).Angular;
        return -sign * w * w * r;
    }

    /// <summary>Adds a force applied at a point: the force on the body and its moment r × F about the center of mass.</summary>
    private void AddPointForce(ReadOnlySpan<double> state, ResolvedPoint point, Vector2d force)
    {
        if (point.Body == Ground)
        {
            return;
        }

        var r = GetPose(state, point.Body).RotateToWorld(point.Local);
        var i = DofPerBody * point.Body;
        forces[i] += force.X;
        forces[i + 1] += force.Y;
        forces[i + 2] += Vector2d.Cross(r, force);
    }

    /// <summary>Adds a pure torque to the body a point belongs to; the ground takes none.</summary>
    private void AddTorque(ResolvedPoint point, double torque)
    {
        if (point.Body != Ground)
        {
            forces[(DofPerBody * point.Body) + 2] += torque;
        }
    }

    private double ActuatorTorque(int actuator, double time, bool useLatchedModes)
    {
        var schedule = actuators[actuator].Schedule;
        return useLatchedModes ? schedule.TorqueInSegment(latchedSegments[actuator], time) : schedule.TorqueAt(time);
    }

    /// <summary>Rate of work done by the actuators, Σ τ·(ω_A − ω_B), W.</summary>
    private double ActuatorPower(double time, ReadOnlySpan<double> state, bool useLatchedModes)
    {
        var total = 0.0;
        for (var a = 0; a < actuators.Length; a++)
        {
            var p = actuatorPin[a];
            total += ActuatorTorque(a, time, useLatchedModes) * (AngularRate(state, pinA[p]) - AngularRate(state, pinB[p]));
        }

        return total;
    }

    private double AngularRate(ReadOnlySpan<double> state, ResolvedPoint point) =>
        point.Body == Ground ? 0.0 : state[CoordinateCount + (DofPerBody * point.Body) + 2];

    /// <summary>Cable length L, rate L̇ and unit direction from end A to end B.</summary>
    private (double Length, double Rate, Vector2d Direction) CableGeometry(ReadOnlySpan<double> state, int cable)
    {
        var span = WorldPoint(state, cableB[cable]) - WorldPoint(state, cableA[cable]);
        var length = span.Length;
        if (!(length > 0))
        {
            throw new InvalidOperationException($"Cable '{cables[cable].Id}' has zero length, so its direction is undefined.");
        }

        var direction = span / length;
        var rate = Vector2d.Dot(direction, PointVelocity(state, cableB[cable]) - PointVelocity(state, cableA[cable]));
        return (length, rate, direction);
    }

    /// <summary>
    /// Rate of energy loss in the cable dampers in the latched modes, W.
    /// Power into a cable is T·L̇ and its stored energy changes at k·Δ·L̇, so
    /// the damper takes (T − k·Δ)·L̇: c·L̇² when taut, −k·Δ·L̇ when recoiling
    /// (positive, since then L̇ &lt; −k·Δ/c &lt; 0), and nothing when slack.
    /// </summary>
    private double DissipatedPower(ReadOnlySpan<double> state)
    {
        var total = 0.0;
        for (var c = 0; c < cables.Length; c++)
        {
            var cable = cables[c];
            var (length, rate, _) = CableGeometry(state, c);
            var stretch = length - cable.RestLength;
            total += latchedModes[c] switch
            {
                CableMode.Taut => cable.Damping * rate * rate,
                CableMode.Recoiling => -cable.Stiffness * stretch * rate,
                _ => 0.0,
            };
        }

        return total;
    }

    private static double ElasticEnergy(Cable cable, double length)
    {
        var stretch = length - cable.RestLength;
        return stretch > 0 ? 0.5 * cable.Stiffness * stretch * stretch : 0.0;
    }

    private CableMode CableModeAt(ReadOnlySpan<double> state, int cable)
    {
        var (length, rate, _) = CableGeometry(state, cable);
        return cables[cable].ModeAt(length, rate);
    }

    private double InverseMass(int coordinate)
    {
        var link = links[coordinate / DofPerBody];
        return coordinate % DofPerBody == 2 ? 1.0 / link.MomentOfInertia : 1.0 / link.Mass;
    }

    private Vector2d WorldPoint(ReadOnlySpan<double> state, ResolvedPoint point) =>
        point.Body == Ground ? point.Local : GetPose(state, point.Body).ToWorld(point.Local);

    private Vector2d PointVelocity(ReadOnlySpan<double> state, ResolvedPoint point)
    {
        if (point.Body == Ground)
        {
            return Vector2d.Zero;
        }

        var pose = GetPose(state, point.Body);
        return GetTwist(state, point.Body).VelocityOfPoint(pose.RotateToWorld(point.Local));
    }

    private ResolvedPoint Resolve(Attachment attachment, string owner)
    {
        if (attachment.IsGround)
        {
            return new ResolvedPoint(Ground, attachment.Point);
        }

        for (var b = 0; b < links.Length; b++)
        {
            if (string.Equals(links[b].Id, attachment.LinkId, StringComparison.Ordinal))
            {
                return new ResolvedPoint(b, attachment.Point);
            }
        }

        throw new ArgumentException($"The {owner} refers to unknown link '{attachment.LinkId}'.");
    }

    private int ResolvePin(JointActuator actuator)
    {
        for (var p = 0; p < pins.Length; p++)
        {
            if (string.Equals(pins[p].Id, actuator.PinId, StringComparison.Ordinal))
            {
                return p;
            }
        }

        throw new ArgumentException($"Actuator '{actuator.Id}' refers to unknown pin '{actuator.PinId}'.");
    }

    private static void RequireUniqueIds(IEnumerable<string> ids, string kind, string paramName)
    {
        var duplicate = ids.GroupBy(id => id, StringComparer.Ordinal).FirstOrDefault(g => g.Count() > 1);
        if (duplicate is not null)
        {
            throw new ArgumentException($"Duplicate {kind} id '{duplicate.Key}'.", paramName);
        }
    }

    private int CheckBody(int body)
    {
        if ((uint)body >= (uint)links.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(body), body, $"System has {links.Length} bodies.");
        }

        return body;
    }

    private void CheckLength(int length)
    {
        if (length != StateLength)
        {
            throw new ArgumentException($"State length {length} does not match system state length {StateLength}.");
        }
    }

    /// <summary>An attachment with its link resolved to a body index (or <see cref="Ground"/>).</summary>
    private readonly record struct ResolvedPoint(int Body, Vector2d Local);
}
