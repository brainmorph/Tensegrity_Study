using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Systems;

/// <summary>
/// A set of planar rigid links under uniform gravity, joined by pin
/// constraints and cables, in maximal coordinates.
///
/// State layout for n bodies and m ground contact points, positions first,
/// then velocities, then the energy dissipated so far, the work done by the
/// joint actuators so far and the work done by external loads and a moving
/// ground so far; then, with a ground line only, its height h and each
/// contact point's tangential spring extension (see <see cref="GroundLine"/>):
/// <code>
/// [ x0 y0 θ0  x1 y1 θ1 … | ẋ0 ẏ0 θ̇0  ẋ1 ẏ1 θ̇1 … | E_diss  W_act  W_ext | h | z0 z1 … ]
/// </code>
/// The ground height is integrated from its schedule's slope, so it is
/// exact, and every energy and contact quantity is a function of the state
/// alone, even under a moving ground.
/// Splitting positions from velocities keeps the generalized-coordinate block
/// contiguous for the constraint solver. E_diss, W_act and W_ext are
/// integrated with the rest of the state, so the energy balance (PLAN.md 4.3)
/// <code>
/// kinetic + gravitational + elastic + dissipated − actuator work − external work = constant
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
/// joint-limit stops between clear, pressing and recoiling (see
/// <see cref="JointLimit"/>), ground contacts between clear, pressing and
/// recoiling and between stick and slip (see <see cref="GroundLine"/>), and
/// actuator and external load schedules between their linear segments (see
/// <see cref="Schedule"/>). As an <see cref="ISwitchedSystem"/>,
/// <see cref="Evaluate"/> uses each element's latched mode, so a system
/// with cables, joint limits, a ground line or non-constant schedules must
/// be stepped by an <see cref="EventSplittingStepper"/> (or
/// have its modes latched by hand). Diagnostics such as
/// <see cref="GetCableTensions"/>, <see cref="GetLimitTorques"/>,
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
    private readonly ContactMode[] latchedLowerStops;
    private readonly ContactMode[] latchedUpperStops;
    private readonly Cable[] cables;
    private readonly ResolvedPoint[] cableA;
    private readonly ResolvedPoint[] cableB;
    private readonly CableMode[] latchedModes;
    private readonly int[] muscleSlot;
    private readonly int[] latchedActivationSegments;
    private readonly JointActuator[] actuators;
    private readonly int[] actuatorPin;
    private readonly int[] latchedSegments;
    private readonly GroundLine? ground;
    private readonly ResolvedPoint[] contacts;
    private readonly ContactMode[] latchedNormals;
    private readonly FrictionMode[] latchedFrictions;
    private readonly ExternalLoad[] loads;
    private readonly ResolvedPoint[] loadPoints;
    private readonly int[] latchedLoadSegments;
    private int latchedGroundSegment;
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
        IEnumerable<JointActuator>? actuators = null,
        GroundLine? ground = null,
        IEnumerable<ExternalLoad>? loads = null,
        double globalDamping = 0)
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
        latchedLowerStops = new ContactMode[this.pins.Length];
        latchedUpperStops = new ContactMode[this.pins.Length];

        this.cables = cables?.ToArray() ?? [];
        RequireUniqueIds(this.cables.Select(c => c.Id), "cable", nameof(cables));
        cableA = this.cables.Select(c => Resolve(c.A, $"cable '{c.Id}'")).ToArray();
        cableB = this.cables.Select(c => Resolve(c.B, $"cable '{c.Id}'")).ToArray();
        latchedModes = new CableMode[this.cables.Length];
        muscleSlot = new int[this.cables.Length];
        var muscleCount = 0;
        for (var c = 0; c < this.cables.Length; c++)
        {
            muscleSlot[c] = this.cables[c].Muscle is null ? -1 : muscleCount++;
        }

        MuscleCount = muscleCount;
        latchedActivationSegments = new int[this.cables.Length];

        this.actuators = actuators?.ToArray() ?? [];
        RequireUniqueIds(this.actuators.Select(a => a.Id), "actuator", nameof(actuators));
        actuatorPin = this.actuators.Select(a => ResolvePin(a.PinId, $"actuator '{a.Id}'")).ToArray();
        var doubled = actuatorPin.GroupBy(p => p).FirstOrDefault(g => g.Count() > 1);
        if (doubled is not null)
        {
            throw new ArgumentException($"Pin '{this.pins[doubled.Key].Id}' has more than one actuator.", nameof(actuators));
        }

        latchedSegments = new int[this.actuators.Length];

        this.ground = ground;
        contacts = ground?.Points.Select(p => Resolve(p.Attachment, $"contact point '{p.Id}'")).ToArray() ?? [];
        latchedNormals = new ContactMode[contacts.Length];
        latchedFrictions = new FrictionMode[contacts.Length];

        if (!double.IsFinite(globalDamping) || globalDamping < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(globalDamping), globalDamping, "Must be finite and non-negative.");
        }

        GlobalDamping = globalDamping;

        this.loads = loads?.ToArray() ?? [];
        RequireUniqueIds(this.loads.Select(l => l.Id), "load", nameof(loads));
        loadPoints = this.loads.Select(l => Resolve(l.Attachment, $"load '{l.Id}'")).ToArray();
        latchedLoadSegments = new int[2 * this.loads.Length];

        // With nothing that switches, the default latch is already correct.
        modesLatched = this.cables.Length == 0
            && this.pins.All(p => p.Limit is null)
            && ground is null
            && this.actuators.All(a => a.Schedule.IsConstant)
            && this.loads.All(l => l.Fx.IsConstant && l.Fy.IsConstant);

        forces = new double[CoordinateCount];
        jacobian = new double[ConstraintCount * CoordinateCount];
        constraintMatrix = new double[ConstraintCount * ConstraintCount];
        multipliers = new double[ConstraintCount];
    }

    public IReadOnlyList<PlanarLink> Links => links;

    public IReadOnlyList<PinJoint> Pins => pins;

    public IReadOnlyList<Cable> Cables => cables;

    public IReadOnlyList<JointActuator> Actuators => actuators;

    /// <summary>External loads (perturbations).</summary>
    public IReadOnlyList<ExternalLoad> Loads => loads;

    /// <summary>The ground line, or null for none.</summary>
    public GroundLine? GroundLine => ground;

    /// <summary>Number of ground contact points.</summary>
    public int ContactCount => contacts.Length;

    public Vector2d Gravity { get; }

    /// <summary>
    /// Global settling damping γ, 1/s (PLAN.md 4.3): each body feels −γ·m·v
    /// and −γ·I·ω, so a free body's speed decays as e^(−γt). Not a physical
    /// element: for settling experiments only, zero in realistic runs, and
    /// recorded in the run manifest. Its loss γ·Σ(m·v² + I·ω²) counts as
    /// dissipated energy.
    /// </summary>
    public double GlobalDamping { get; }

    public ConstraintStabilization Stabilization { get; }

    /// <summary>Number of scalar constraint equations: 2 per pin.</summary>
    public int ConstraintCount => 2 * pins.Length;

    public int BodyCount => links.Length;

    /// <summary>Number of generalized coordinates: 3 per body.</summary>
    public int CoordinateCount => DofPerBody * links.Length;

    /// <summary>
    /// Length of the state vector: coordinates, velocities, dissipated
    /// energy, actuator work, external work, then ground height and contact
    /// extensions if there is a ground, then one contractile length per
    /// muscle. A system with no ground and no muscles ends at external work.
    /// </summary>
    public int StateLength => MuscleBase + MuscleCount;

    /// <summary>Number of cables with a <see cref="Muscle"/> in series.</summary>
    public int MuscleCount { get; }

    /// <summary>Index of the contractile length ℓ of cable <paramref name="cable"/>'s muscle in the state vector.</summary>
    /// <exception cref="ArgumentException">The cable has no muscle.</exception>
    public int ContractileLengthIndex(int cable)
    {
        if ((uint)cable >= (uint)cables.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(cable), cable, $"System has {cables.Length} cables.");
        }

        var slot = muscleSlot[cable];
        return slot >= 0
            ? MuscleBase + slot
            : throw new ArgumentException($"Cable '{cables[cable].Id}' has no muscle.", nameof(cable));
    }

    /// <summary>Contractile length ℓ of cable <paramref name="cable"/>'s muscle at this state, m.</summary>
    public double GetContractileLength(ReadOnlySpan<double> state, int cable)
    {
        CheckLength(state.Length);
        return state[ContractileLengthIndex(cable)];
    }

    /// <summary>Index of the cumulative dissipated energy in the state vector.</summary>
    public int DissipatedEnergyIndex => 2 * CoordinateCount;

    /// <summary>Index of the cumulative actuator work in the state vector.</summary>
    public int ActuatorWorkIndex => (2 * CoordinateCount) + 1;

    /// <summary>Index of the cumulative work done by external loads in the state vector.</summary>
    public int ExternalWorkIndex => (2 * CoordinateCount) + 2;

    /// <summary>Index of contact point <paramref name="contact"/>'s tangential spring extension z in the state vector.</summary>
    public int ExtensionIndex(int contact) => ExtensionBase + CheckContact(contact);

    /// <summary>Index of the ground line's height h in the state vector.</summary>
    /// <exception cref="InvalidOperationException">The system has no ground line.</exception>
    public int GroundHeightIndex => ground is not null
        ? (2 * CoordinateCount) + 3
        : throw new InvalidOperationException("The system has no ground line.");

    private int ExtensionBase => (2 * CoordinateCount) + 3 + (ground is null ? 0 : 1);

    private int MuscleBase => ExtensionBase + contacts.Length;

    public int PositionIndex(int body) => DofPerBody * CheckBody(body);

    public int VelocityIndex(int body) => CoordinateCount + (DofPerBody * CheckBody(body));

    /// <summary>
    /// A zero state at <paramref name="time"/>, with the ground (if any) at
    /// its scheduled height and each muscle at its contractile length then.
    /// </summary>
    public SystemState CreateState(double time = 0)
    {
        var state = new SystemState(StateLength, time);
        if (ground is not null)
        {
            state.Values[GroundHeightIndex] = ground.HeightSchedule.ValueAt(time);
        }

        for (var c = 0; c < cables.Length; c++)
        {
            if (cables[c].Muscle is { } muscle)
            {
                state.Values[MuscleBase + muscleSlot[c]] = muscle.LengthAt(time);
            }
        }

        return state;
    }

    /// <summary>Height of the ground line at this state, m.</summary>
    public double GetGroundHeight(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[GroundHeightIndex];
    }

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
                "Modes are not latched. Step a system with cables, joint limits, a ground line or scheduled actuators through an EventSplittingStepper, or call LatchModes first.");
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

        derivative[DissipatedEnergyIndex] = DissipatedPower(time, state);
        derivative[ActuatorWorkIndex] = ActuatorPower(time, state, useLatchedModes: true);
        derivative[ExternalWorkIndex] = ExternalPower(time, state, useLatchedModes: true);

        if (ground is not null)
        {
            derivative[GroundHeightIndex] = GroundRate(time, useLatchedModes: true);
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            var c = ContactAt(time, state, i, useLatchedModes: true);
            derivative[ExtensionIndex(i)] = ground!.ExtensionRate(c.Friction, c.NormalForce, c.Extension, c.Velocity.X);
        }

        for (var c = 0; c < cables.Length; c++)
        {
            if (muscleSlot[c] >= 0)
            {
                derivative[MuscleBase + muscleSlot[c]] = ContractileRate(c, time, useLatchedModes: true);
            }
        }
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
    /// Ground contact and external load forces count like cable forces, by
    /// where their point sits. Uses the physical cable law, ground contact law,
    /// joint-limit torques and actuator torques at the given time and state.
    /// Limit and actuator torques are pure couples, so they enter only
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
            var (length, rate, direction) = TendonGeometry(time, state, c, useLatchedModes: false);
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

        for (var i = 0; i < contacts.Length; i++)
        {
            AddToHalf(halfForce, contacts[i], ContactAt(time, state, i, useLatchedModes: false).Force);
        }

        for (var l = 0; l < loads.Length; l++)
        {
            AddToHalf(halfForce, loadPoints[l], loads[l].ForceAt(time));
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

    /// <summary>
    /// The joint torques at <paramref name="pinIds"/> that hold this state
    /// still: test 11's effort measure (PLAN.md 7). Applied loads are taken
    /// at <paramref name="time"/> with the physical cable and joint-limit
    /// laws, and include any actuators the system already has.
    ///
    /// The constrained acceleration is affine in the added torques,
    /// q̈ = q̈₀ + G·τ, where column j of G is the response to a unit couple
    /// at pin j (found with the same constraint solve as the dynamics). The
    /// torques minimize ½·q̈ᵀ·M·q̈, the kinetic-energy measure of the
    /// leftover acceleration, by the normal equations
    /// <code>
    /// (Gᵀ·M·G)·τ = −Gᵀ·M·q̈₀
    /// </code>
    /// When the pins can hold the state the minimum is zero and the torques
    /// are exact; otherwise the leftover acceleration is reported. Meant for
    /// a state at rest, where q̈ = 0 means the structure stays put; with
    /// velocities it also cancels the velocity terms for that instant.
    ///
    /// Redundant pins, as in a closed loop with every joint actuated, have
    /// many torque sets that give the same acceleration, and Gᵀ·M·G is
    /// singular. Then the torques are the set with the smallest Σ τ², the
    /// pseudo-inverse solution, and <see cref="HoldingTorques.Redundancy"/>
    /// says how many torque combinations do nothing.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// A pin's torque moves nothing (the structure is rigid there).
    /// </exception>
    public HoldingTorques SolveHoldingTorques(double time, ReadOnlySpan<double> state, IReadOnlyList<string> pinIds)
    {
        CheckLength(state.Length);
        ArgumentNullException.ThrowIfNull(pinIds);
        RequireUniqueIds(pinIds, "pin", nameof(pinIds));
        var joints = pinIds.Select(id => ResolvePin(id, "holding torque")).ToArray();
        var n = CoordinateCount;
        var m = joints.Length;

        ComputeTotalForces(time, state, useLatchedModes: false);
        var baseline = new double[n];
        for (var k = 0; k < n; k++)
        {
            baseline[k] = forces[k] * InverseMass(k);
        }

        // Column j of G, stored as row j: the acceleration from a unit couple at pin j.
        var response = new double[m * n];
        for (var j = 0; j < m; j++)
        {
            var column = response.AsSpan(j * n, n);
            AddUnitCouple(column, pinA[joints[j]], 1.0);
            AddUnitCouple(column, pinB[joints[j]], -1.0);
            ConstrainedAcceleration(column);
        }

        var normal = new double[m * m];
        var torques = new double[m];
        for (var i = 0; i < m; i++)
        {
            var gi = response.AsSpan(i * n, n);
            for (var k = 0; k < n; k++)
            {
                torques[i] -= gi[k] * Mass(k) * baseline[k];
            }

            for (var j = 0; j <= i; j++)
            {
                var gj = response.AsSpan(j * n, n);
                var sum = 0.0;
                for (var k = 0; k < n; k++)
                {
                    sum += gi[k] * Mass(k) * gj[k];
                }

                normal[(i * m) + j] = sum;
            }
        }

        // A couple whose response is round-off next to an unconstrained
        // body's (Σ 1/I over the bodies it acts on) does nothing: the joint
        // is locked by the pins themselves, as in a rigid triangle.
        for (var j = 0; j < m; j++)
        {
            var free = UnconstrainedCoupleResponse(pinA[joints[j]]) + UnconstrainedCoupleResponse(pinB[joints[j]]);
            if (!(normal[(j * m) + j] > 1e-12 * free))
            {
                throw new InvalidOperationException(
                    $"A torque at pin '{pinIds[j]}' cannot move anything at this state, so it has no holding torque.");
            }
        }

        var redundancy = 0;
        if (m > 0)
        {
            // Cholesky needs the lower triangle only and overwrites it, so
            // keep a copy for the redundant case.
            var copy = normal.ToArray();
            if (DenseCholesky.TryFactor(normal, m))
            {
                DenseCholesky.Solve(normal, m, torques);
            }
            else
            {
                redundancy = m - SymmetricEigen.SolveMinimumNorm(copy, m, torques);
            }
        }

        double maxLinear = 0, maxAngular = 0;
        Span<double> a = stackalloc double[DofPerBody];
        for (var b = 0; b < links.Length; b++)
        {
            for (var d = 0; d < DofPerBody; d++)
            {
                var k = (DofPerBody * b) + d;
                a[d] = baseline[k];
                for (var j = 0; j < m; j++)
                {
                    a[d] += response[(j * n) + k] * torques[j];
                }
            }

            maxLinear = Math.Max(maxLinear, Math.Sqrt((a[0] * a[0]) + (a[1] * a[1])));
            maxAngular = Math.Max(maxAngular, Math.Abs(a[2]));
        }

        return new HoldingTorques(pinIds.ToArray(), torques, maxLinear, maxAngular, redundancy);
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
            latchedModes[c] = CableModeAt(time, state, c);
            latchedActivationSegments[c] = cables[c].Muscle?.Activation.SegmentAt(time) ?? 0;
        }

        for (var p = 0; p < pins.Length; p++)
        {
            (latchedLowerStops[p], latchedUpperStops[p]) = LimitModesAt(state, p);
        }

        if (ground is not null)
        {
            latchedGroundSegment = ground.HeightSchedule.SegmentAt(time);
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            var c = ContactAt(time, state, i, useLatchedModes: false);
            latchedNormals[i] = c.Normal;
            latchedFrictions[i] = c.Friction;
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            latchedSegments[a] = actuators[a].Schedule.SegmentAt(time);
        }

        for (var l = 0; l < loads.Length; l++)
        {
            latchedLoadSegments[2 * l] = loads[l].Fx.SegmentAt(time);
            latchedLoadSegments[(2 * l) + 1] = loads[l].Fy.SegmentAt(time);
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
            if (CableModeAt(time, state, c) != latchedModes[c])
            {
                return false;
            }

            if (cables[c].Muscle is { } muscle && muscle.Activation.SegmentAt(time) != latchedActivationSegments[c])
            {
                return false;
            }
        }

        for (var p = 0; p < pins.Length; p++)
        {
            if (LimitModesAt(state, p) != (latchedLowerStops[p], latchedUpperStops[p]))
            {
                return false;
            }
        }

        if (ground is not null && ground.HeightSchedule.SegmentAt(time) != latchedGroundSegment)
        {
            return false;
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            var c = ContactAt(time, state, i, useLatchedModes: false);
            if (c.Normal != latchedNormals[i] || c.Friction != latchedFrictions[i])
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

        for (var l = 0; l < loads.Length; l++)
        {
            if (loads[l].Fx.SegmentAt(time) != latchedLoadSegments[2 * l] || loads[l].Fy.SegmentAt(time) != latchedLoadSegments[(2 * l) + 1])
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

    /// <summary>The physical mode of plain cable <paramref name="cable"/> at this state.</summary>
    /// <exception cref="InvalidOperationException">The cable has a muscle, whose mode depends on the time; use the overload that takes one.</exception>
    public CableMode GetCableMode(ReadOnlySpan<double> state, int cable)
    {
        var (length, rate) = GetCableLength(state, cable);
        if (cables[cable].Muscle is not null)
        {
            throw new InvalidOperationException($"Cable '{cables[cable].Id}' has a muscle, so its mode depends on the time.");
        }

        return cables[cable].ModeAt(length, rate);
    }

    /// <summary>
    /// The physical mode of cable <paramref name="cable"/> at this time and
    /// state. For a muscle's tendon it is the tendon's own mode, which
    /// depends on the muscle's contractile rate at <paramref name="time"/>.
    /// </summary>
    public CableMode GetCableMode(double time, ReadOnlySpan<double> state, int cable)
    {
        GetCableLength(state, cable);
        return CableModeAt(time, state, cable);
    }

    /// <summary>Physical tension of every cable at this state, N.</summary>
    /// <exception cref="InvalidOperationException">A cable has a muscle, whose tension depends on the time; use the overload that takes one.</exception>
    public void GetCableTensions(ReadOnlySpan<double> state, Span<double> tensions)
    {
        if (MuscleCount > 0)
        {
            throw new InvalidOperationException("The system has muscles, so cable tensions depend on the time.");
        }

        GetCableTensions(0, state, tensions);
    }

    /// <summary>
    /// Physical tension of every cable at this time and state, N. A muscle's
    /// tendon damper sees the tendon's own rate, which depends on the
    /// muscle's contractile rate at <paramref name="time"/>.
    /// </summary>
    public void GetCableTensions(double time, ReadOnlySpan<double> state, Span<double> tensions)
    {
        CheckLength(state.Length);
        if (tensions.Length != cables.Length)
        {
            throw new ArgumentException($"Expected {cables.Length} tensions, got {tensions.Length}.", nameof(tensions));
        }

        for (var c = 0; c < cables.Length; c++)
        {
            var (length, rate, _) = TendonGeometry(time, state, c, useLatchedModes: false);
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
            torques[a] = actuators[a].Schedule.ValueAt(time);
        }
    }

    /// <summary>
    /// Joint angle φ = θ_A − θ_B of pin <paramref name="pin"/> and its rate
    /// φ̇, rad and rad/s. A ground end counts as θ = 0.
    /// </summary>
    public (double Angle, double Rate) GetJointAngle(ReadOnlySpan<double> state, int pin)
    {
        CheckLength(state.Length);
        CheckPin(pin);
        return (JointAngle(state, pin), RelativeRate(state, pin));
    }

    /// <summary>Physical modes of pin <paramref name="pin"/>'s lower and upper stops; both clear without a limit.</summary>
    public (ContactMode Lower, ContactMode Upper) GetLimitModes(ReadOnlySpan<double> state, int pin)
    {
        CheckLength(state.Length);
        CheckPin(pin);
        return LimitModesAt(state, pin);
    }

    /// <summary>
    /// Physical joint-limit torque on each pin's end A at this state, N·m
    /// (end B takes the opposite); zero for a pin without a limit.
    /// </summary>
    public void GetLimitTorques(ReadOnlySpan<double> state, Span<double> torques)
    {
        CheckLength(state.Length);
        if (torques.Length != pins.Length)
        {
            throw new ArgumentException($"Expected {pins.Length} torques, got {torques.Length}.", nameof(torques));
        }

        for (var p = 0; p < pins.Length; p++)
        {
            torques[p] = pins[p].Limit?.Torque(JointAngle(state, p), RelativeRate(state, p)) ?? 0.0;
        }
    }

    /// <summary>
    /// Physical force the ground exerts on each contact point at this state,
    /// N: friction along x, the normal force along y.
    /// </summary>
    public void GetContactForces(double time, ReadOnlySpan<double> state, Span<Vector2d> forces)
    {
        CheckLength(state.Length);
        if (forces.Length != contacts.Length)
        {
            throw new ArgumentException($"Expected {contacts.Length} forces, got {forces.Length}.", nameof(forces));
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            forces[i] = ContactAt(time, state, i, useLatchedModes: false).Force;
        }
    }

    /// <summary>Physical normal and tangential modes of contact point <paramref name="contact"/> at this state.</summary>
    public (ContactMode Normal, FrictionMode Friction) GetContactModes(double time, ReadOnlySpan<double> state, int contact)
    {
        CheckLength(state.Length);
        CheckContact(contact);
        var c = ContactAt(time, state, contact, useLatchedModes: false);
        return (c.Normal, c.Friction);
    }

    /// <summary>World position of contact point <paramref name="contact"/> at this state.</summary>
    public Vector2d GetContactPoint(ReadOnlySpan<double> state, int contact)
    {
        CheckLength(state.Length);
        return WorldPoint(state, contacts[CheckContact(contact)]);
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

    /// <summary>Elastic energy stored in the cables, Σ ½·k·(L − L0)² over stretched cables (a muscle's tendon with its own L), J.</summary>
    public double CableElasticEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var total = 0.0;
        for (var c = 0; c < cables.Length; c++)
        {
            var length = CableGeometry(state, c).Length;
            if (muscleSlot[c] >= 0)
            {
                length -= state[MuscleBase + muscleSlot[c]];
            }

            total += ElasticEnergy(cables[c], length);
        }

        return total;
    }

    /// <summary>Elastic energy stored in compressed joint-limit stops, Σ ½·k·δ², J.</summary>
    public double LimitElasticEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var total = 0.0;
        for (var p = 0; p < pins.Length; p++)
        {
            if (pins[p].Limit is { } limit)
            {
                total += limit.ElasticEnergy(JointAngle(state, p));
            }
        }

        return total;
    }

    /// <summary>
    /// Elastic energy stored in the ground contacts, J: ½·k·δ² in the
    /// normal springs of points below the line plus ½·k_t·z² in every
    /// tangential spring.
    /// </summary>
    public double GroundElasticEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        var total = 0.0;
        for (var i = 0; i < contacts.Length; i++)
        {
            total += ground!.Normal.ElasticEnergy(GroundLine.Penetration(state[GroundHeightIndex], WorldPoint(state, contacts[i]).Y))
                + ground.TangentialElasticEnergy(state[ExtensionIndex(i)]);
        }

        return total;
    }

    /// <summary>All stored elastic energy: cables, joint-limit stops and ground contacts, J.</summary>
    public double ElasticEnergy(ReadOnlySpan<double> state) =>
        CableElasticEnergy(state) + LimitElasticEnergy(state) + GroundElasticEnergy(state);

    /// <summary>Energy dissipated by cable damping, joint-limit damping, joint friction, ground contact and global damping since the start of the run, J.</summary>
    public double DissipatedEnergy(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[DissipatedEnergyIndex];
    }

    /// <summary>Work done on the system by external loads since the start of the run, J.</summary>
    public double ExternalWork(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[ExternalWorkIndex];
    }

    /// <summary>Force of every external load at <paramref name="time"/>, N (world frame), from its schedule.</summary>
    public void GetLoadForces(double time, Span<Vector2d> forces)
    {
        if (forces.Length != loads.Length)
        {
            throw new ArgumentException($"Expected {loads.Length} forces, got {forces.Length}.", nameof(forces));
        }

        for (var l = 0; l < loads.Length; l++)
        {
            forces[l] = loads[l].ForceAt(time);
        }
    }

    /// <summary>Work done on the system by the joint actuators and muscles since the start of the run, J.</summary>
    public double ActuatorWork(ReadOnlySpan<double> state)
    {
        CheckLength(state.Length);
        return state[ActuatorWorkIndex];
    }

    /// <summary>Kinetic + gravitational + elastic (cables, joint limits and ground) energy, J.</summary>
    public double MechanicalEnergy(ReadOnlySpan<double> state) =>
        KineticEnergy(state) + GravitationalPotentialEnergy(state) + ElasticEnergy(state);

    /// <summary>
    /// Mechanical energy plus energy dissipated so far, minus the work the
    /// actuators and external loads have put in, J. It is conserved, so its
    /// drift measures the integration error (PLAN.md 4.3, test 7).
    /// </summary>
    public double TotalEnergy(ReadOnlySpan<double> state) =>
        MechanicalEnergy(state) + DissipatedEnergy(state) - ActuatorWork(state) - ExternalWork(state);

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

            if (GlobalDamping > 0)
            {
                var v = CoordinateCount + i;
                forces[i] -= GlobalDamping * link.Mass * state[v];
                forces[i + 1] -= GlobalDamping * link.Mass * state[v + 1];
                forces[i + 2] -= GlobalDamping * link.MomentOfInertia * state[v + 2];
            }
        }

        for (var c = 0; c < cables.Length; c++)
        {
            var cable = cables[c];
            var (length, rate, direction) = TendonGeometry(time, state, c, useLatchedModes);
            var mode = useLatchedModes ? latchedModes[c] : cable.ModeAt(length, rate);
            var tension = cable.TensionInMode(mode, length, rate);

            // The cable pulls end A toward B and end B toward A.
            AddPointForce(state, cableA[c], tension * direction);
            AddPointForce(state, cableB[c], -tension * direction);
        }

        for (var l = 0; l < loads.Length; l++)
        {
            AddPointForce(state, loadPoints[l], LoadForce(l, time, useLatchedModes));
        }

        for (var a = 0; a < actuators.Length; a++)
        {
            var torque = ActuatorTorque(a, time, useLatchedModes);
            var p = actuatorPin[a];
            AddTorque(pinA[p], torque);
            AddTorque(pinB[p], -torque);
        }

        for (var p = 0; p < pins.Length; p++)
        {
            var friction = pins[p].Friction;
            if (friction > 0)
            {
                var torque = friction * RelativeRate(state, p);
                AddTorque(pinA[p], -torque);
                AddTorque(pinB[p], torque);
            }

            if (pins[p].Limit is { } limit)
            {
                var angle = JointAngle(state, p);
                var rate = RelativeRate(state, p);
                var torque = useLatchedModes
                    ? limit.TorqueInModes(latchedLowerStops[p], latchedUpperStops[p], angle, rate)
                    : limit.Torque(angle, rate);
                AddTorque(pinA[p], torque);
                AddTorque(pinB[p], -torque);
            }
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            AddPointForce(state, contacts[i], ContactAt(time, state, i, useLatchedModes).Force);
        }

        if (pins.Length > 0)
        {
            AddConstraintForces(state);
        }
    }

    /// <summary>
    /// Everything about contact point <paramref name="contact"/> at this
    /// state: its velocity and extension, its normal and tangential modes
    /// (latched or physical), and the force the ground exerts on it. The
    /// penetration rate is δ̇ = ḣ − ẏ, with ḣ from the height schedule.
    /// </summary>
    private ContactState ContactAt(double time, ReadOnlySpan<double> state, int contact, bool useLatchedModes)
    {
        var g = ground!;
        var point = contacts[contact];
        var position = WorldPoint(state, point);
        var velocity = PointVelocity(state, point);
        var extension = state[ExtensionBase + contact];
        var penetration = GroundLine.Penetration(state[GroundHeightIndex], position.Y);
        var rate = g.IsMoving ? GroundRate(time, useLatchedModes) - velocity.Y : -velocity.Y;

        ContactMode normal;
        FrictionMode friction;
        double normalForce;
        if (useLatchedModes)
        {
            normal = latchedNormals[contact];
            friction = latchedFrictions[contact];
            normalForce = g.Normal.ForceInMode(normal, penetration, rate);
        }
        else
        {
            normal = g.Normal.ModeAt(penetration, rate);
            normalForce = g.Normal.ForceInMode(normal, penetration, rate);
            friction = g.FrictionModeAt(normal, normalForce, extension, velocity.X);
        }

        var tangential = g.FrictionForce(friction, normalForce, extension, velocity.X);
        return new ContactState(normal, friction, penetration, rate, normalForce, extension, velocity, new Vector2d(tangential, normalForce));
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

    private double UnconstrainedCoupleResponse(ResolvedPoint point) =>
        point.Body == Ground ? 0.0 : 1.0 / links[point.Body].MomentOfInertia;

    private static void AddUnitCouple(Span<double> generalized, ResolvedPoint point, double sign)
    {
        if (point.Body != Ground)
        {
            generalized[(DofPerBody * point.Body) + 2] += sign;
        }
    }

    /// <summary>
    /// Replaces the generalized force <paramref name="load"/> with the
    /// acceleration it causes once the pins react: M⁻¹·(f + Jᵀ·μ) with
    /// (J·M⁻¹·Jᵀ)·μ = −J·M⁻¹·f. Linear in f, with no stabilization terms.
    /// Uses the Jacobian and factor left by the last
    /// <see cref="ComputeTotalForces"/>, which must have run at this state.
    /// </summary>
    private void ConstrainedAcceleration(Span<double> load)
    {
        var rows = ConstraintCount;
        var cols = CoordinateCount;
        if (rows > 0)
        {
            Span<double> mu = rows <= 64 ? stackalloc double[rows] : new double[rows];
            for (var r = 0; r < rows; r++)
            {
                var jr = jacobian.AsSpan(r * cols, cols);
                var sum = 0.0;
                for (var k = 0; k < cols; k++)
                {
                    sum += jr[k] * InverseMass(k) * load[k];
                }

                mu[r] = -sum;
            }

            DenseCholesky.Solve(constraintMatrix, rows, mu);
            for (var r = 0; r < rows; r++)
            {
                var jr = jacobian.AsSpan(r * cols, cols);
                for (var k = 0; k < cols; k++)
                {
                    load[k] += jr[k] * mu[r];
                }
            }
        }

        for (var k = 0; k < cols; k++)
        {
            load[k] *= InverseMass(k);
        }
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
        return useLatchedModes ? schedule.ValueInSegment(latchedSegments[actuator], time) : schedule.ValueAt(time);
    }

    /// <summary>
    /// Rate of work done by the actuators, Σ τ·(ω_A − ω_B), and by the
    /// muscles, Σ −T·ℓ̇, W. A muscle's power balances its tendon: the path
    /// takes −T·L̇_path from the bodies, the tendon stores and its damper
    /// loses T·(L̇_path − ℓ̇), so the muscle supplies the difference −T·ℓ̇.
    /// </summary>
    private double ActuatorPower(double time, ReadOnlySpan<double> state, bool useLatchedModes)
    {
        var total = 0.0;
        for (var a = 0; a < actuators.Length; a++)
        {
            var p = actuatorPin[a];
            total += ActuatorTorque(a, time, useLatchedModes) * RelativeRate(state, p);
        }

        for (var c = 0; c < cables.Length; c++)
        {
            if (muscleSlot[c] < 0)
            {
                continue;
            }

            var cable = cables[c];
            var (length, rate, _) = TendonGeometry(time, state, c, useLatchedModes);
            var mode = useLatchedModes ? latchedModes[c] : cable.ModeAt(length, rate);
            total -= cable.TensionInMode(mode, length, rate) * ContractileRate(c, time, useLatchedModes);
        }

        return total;
    }

    /// <summary>Contractile rate ℓ̇ of cable <paramref name="cable"/>'s muscle, m/s, from the latched or current activation segment.</summary>
    private double ContractileRate(int cable, double time, bool useLatchedModes)
    {
        var muscle = cables[cable].Muscle!;
        return useLatchedModes ? muscle.RateInSegment(latchedActivationSegments[cable], time) : muscle.RateAt(time);
    }

    /// <summary>
    /// Length, rate and direction of cable <paramref name="cable"/>'s
    /// tendon: the cable's own for a plain cable, and the path less the
    /// muscle's contractile part, L − ℓ and L̇ − ℓ̇, for a muscle's tendon.
    /// </summary>
    private (double Length, double Rate, Vector2d Direction) TendonGeometry(double time, ReadOnlySpan<double> state, int cable, bool useLatchedModes)
    {
        var path = CableGeometry(state, cable);
        var slot = muscleSlot[cable];
        if (slot < 0)
        {
            return path;
        }

        return (path.Length - state[MuscleBase + slot], path.Rate - ContractileRate(cable, time, useLatchedModes), path.Direction);
    }

    private Vector2d LoadForce(int load, double time, bool useLatchedModes)
    {
        var l = loads[load];
        return useLatchedModes
            ? new Vector2d(l.Fx.ValueInSegment(latchedLoadSegments[2 * load], time), l.Fy.ValueInSegment(latchedLoadSegments[(2 * load) + 1], time))
            : l.ForceAt(time);
    }

    /// <summary>
    /// Rate of work done by the external loads, Σ F·v at each load's point,
    /// and by a moving ground, Σ N·ḣ over the contact points, W.
    /// </summary>
    private double ExternalPower(double time, ReadOnlySpan<double> state, bool useLatchedModes)
    {
        var total = 0.0;
        for (var l = 0; l < loads.Length; l++)
        {
            total += Vector2d.Dot(LoadForce(l, time, useLatchedModes), PointVelocity(state, loadPoints[l]));
        }

        if (ground is { IsMoving: true })
        {
            var rate = GroundRate(time, useLatchedModes);
            for (var i = 0; i < contacts.Length; i++)
            {
                total += ContactAt(time, state, i, useLatchedModes).NormalForce * rate;
            }
        }

        return total;
    }

    /// <summary>Rate ḣ at which the ground line rises, m/s, from the latched or current schedule segment.</summary>
    private double GroundRate(double time, bool useLatchedModes) =>
        useLatchedModes ? ground!.HeightSchedule.SlopeInSegment(latchedGroundSegment) : ground!.HeightSchedule.SlopeAt(time);

    private double AngularRate(ReadOnlySpan<double> state, ResolvedPoint point) =>
        point.Body == Ground ? 0.0 : state[CoordinateCount + (DofPerBody * point.Body) + 2];

    private double BodyAngle(ReadOnlySpan<double> state, ResolvedPoint point) =>
        point.Body == Ground ? 0.0 : state[(DofPerBody * point.Body) + 2];

    /// <summary>Joint angle θ_A − θ_B across pin <paramref name="pin"/>, rad.</summary>
    private double JointAngle(ReadOnlySpan<double> state, int pin) =>
        BodyAngle(state, pinA[pin]) - BodyAngle(state, pinB[pin]);

    private (ContactMode Lower, ContactMode Upper) LimitModesAt(ReadOnlySpan<double> state, int pin)
    {
        if (pins[pin].Limit is not { } limit)
        {
            return (ContactMode.Clear, ContactMode.Clear);
        }

        var angle = JointAngle(state, pin);
        var rate = RelativeRate(state, pin);
        return (limit.LowerMode(angle, rate), limit.UpperMode(angle, rate));
    }

    /// <summary>Relative rate ω_A − ω_B across pin <paramref name="pin"/>, rad/s.</summary>
    private double RelativeRate(ReadOnlySpan<double> state, int pin) =>
        AngularRate(state, pinA[pin]) - AngularRate(state, pinB[pin]);

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
    /// Rate of energy loss in the cable and joint-limit dampers in the
    /// latched modes and in joint friction, W. Power into a cable is T·L̇
    /// and its stored energy changes at k·Δ·L̇, so the damper takes
    /// (T − k·Δ)·L̇: c·L̇² when taut, −k·Δ·L̇ when recoiling (positive, since
    /// then L̇ &lt; −k·Δ/c &lt; 0), and nothing when slack. Joint-limit stops
    /// follow the same rule (<see cref="ContactLaw.DissipatedPowerInMode"/>),
    /// as does each ground contact's normal spring; its tangential damper
    /// and slider take <see cref="GroundLine.TangentialDissipatedPower"/>.
    /// A pin with friction c takes c·Δω², and global damping γ takes
    /// γ·Σ(m·v² + I·ω²) = 2γ·KE.
    /// </summary>
    private double DissipatedPower(double time, ReadOnlySpan<double> state)
    {
        var total = GlobalDamping > 0 ? 2 * GlobalDamping * KineticEnergy(state) : 0.0;
        for (var c = 0; c < cables.Length; c++)
        {
            var cable = cables[c];
            var (length, rate, _) = TendonGeometry(time, state, c, useLatchedModes: true);
            var stretch = length - cable.RestLength;
            total += latchedModes[c] switch
            {
                CableMode.Taut => cable.Damping * rate * rate,
                CableMode.Recoiling => -cable.Stiffness * stretch * rate,
                _ => 0.0,
            };
        }

        for (var p = 0; p < pins.Length; p++)
        {
            var friction = pins[p].Friction;
            if (friction > 0)
            {
                var rate = RelativeRate(state, p);
                total += friction * rate * rate;
            }

            if (pins[p].Limit is { } limit)
            {
                total += limit.DissipatedPowerInModes(latchedLowerStops[p], latchedUpperStops[p], JointAngle(state, p), RelativeRate(state, p));
            }
        }

        for (var i = 0; i < contacts.Length; i++)
        {
            var c = ContactAt(time, state, i, useLatchedModes: true);
            total += ground!.Normal.DissipatedPowerInMode(c.Normal, c.Penetration, c.PenetrationRate)
                + ground.TangentialDissipatedPower(c.Friction, c.NormalForce, c.Extension, c.Velocity.X);
        }

        return total;
    }

    private static double ElasticEnergy(Cable cable, double length)
    {
        var stretch = length - cable.RestLength;
        return stretch > 0 ? 0.5 * cable.Stiffness * stretch * stretch : 0.0;
    }

    private CableMode CableModeAt(double time, ReadOnlySpan<double> state, int cable)
    {
        var (length, rate, _) = TendonGeometry(time, state, cable, useLatchedModes: false);
        return cables[cable].ModeAt(length, rate);
    }

    private double Mass(int coordinate)
    {
        var link = links[coordinate / DofPerBody];
        return coordinate % DofPerBody == 2 ? link.MomentOfInertia : link.Mass;
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

    private int ResolvePin(string pinId, string owner)
    {
        for (var p = 0; p < pins.Length; p++)
        {
            if (string.Equals(pins[p].Id, pinId, StringComparison.Ordinal))
            {
                return p;
            }
        }

        throw new ArgumentException($"The {owner} refers to unknown pin '{pinId}'.");
    }

    private static void RequireUniqueIds(IEnumerable<string> ids, string kind, string paramName)
    {
        var duplicate = ids.GroupBy(id => id, StringComparer.Ordinal).FirstOrDefault(g => g.Count() > 1);
        if (duplicate is not null)
        {
            throw new ArgumentException($"Duplicate {kind} id '{duplicate.Key}'.", paramName);
        }
    }

    private int CheckContact(int contact)
    {
        if ((uint)contact >= (uint)contacts.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(contact), contact, $"System has {contacts.Length} contact points.");
        }

        return contact;
    }

    private void CheckPin(int pin)
    {
        if ((uint)pin >= (uint)pins.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(pin), pin, $"System has {pins.Length} pins.");
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

    private readonly record struct ContactState(
        ContactMode Normal,
        FrictionMode Friction,
        double Penetration,
        double PenetrationRate,
        double NormalForce,
        double Extension,
        Vector2d Velocity,
        Vector2d Force);
}
