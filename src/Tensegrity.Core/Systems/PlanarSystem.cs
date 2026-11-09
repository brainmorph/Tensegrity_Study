using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Systems;

/// <summary>
/// A set of planar rigid links under uniform gravity, joined by pin
/// constraints and cables, in maximal coordinates.
///
/// State layout for n bodies (length 6n), positions first, then velocities:
/// <code>
/// [ x0 y0 θ0  x1 y1 θ1 … | ẋ0 ẏ0 θ̇0  ẋ1 ẏ1 θ̇1 … ]
/// </code>
/// Splitting positions from velocities keeps the generalized-coordinate block
/// contiguous for the constraint solver.
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
/// Cables switch between slack, taut and recoiling (see <see cref="Cable"/>).
/// As an <see cref="ISwitchedSystem"/>, <see cref="Evaluate"/> uses each
/// cable's latched mode, so a system with cables must be stepped by an
/// <see cref="EventSplittingStepper"/> (or have its modes latched by hand).
/// Diagnostics such as <see cref="GetCableTensions"/> and
/// <see cref="GetPinReactions"/> always use the physical cable law at the
/// given state, independent of the latch.
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
        ConstraintStabilization? stabilization = null)
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
        modesLatched = this.cables.Length == 0;

        forces = new double[CoordinateCount];
        jacobian = new double[ConstraintCount * CoordinateCount];
        constraintMatrix = new double[ConstraintCount * ConstraintCount];
        multipliers = new double[ConstraintCount];
    }

    public IReadOnlyList<PlanarLink> Links => links;

    public IReadOnlyList<PinJoint> Pins => pins;

    public IReadOnlyList<Cable> Cables => cables;

    public Vector2d Gravity { get; }

    public ConstraintStabilization Stabilization { get; }

    /// <summary>Number of scalar constraint equations: 2 per pin.</summary>
    public int ConstraintCount => 2 * pins.Length;

    public int BodyCount => links.Length;

    /// <summary>Number of generalized coordinates: 3 per body.</summary>
    public int CoordinateCount => DofPerBody * links.Length;

    /// <summary>Length of the state vector: coordinates plus velocities.</summary>
    public int StateLength => 2 * CoordinateCount;

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
        state[n..].CopyTo(derivative[..n]);

        if (!modesLatched)
        {
            throw new InvalidOperationException(
                "Cable modes are not latched. Step a system with cables through an EventSplittingStepper, or call LatchModes first.");
        }

        ComputeTotalForces(state, useLatchedModes: true);

        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var i = DofPerBody * b;
            derivative[n + i] = forces[i] / link.Mass;
            derivative[n + i + 1] = forces[i + 1] / link.Mass;
            derivative[n + i + 2] = forces[i + 2] / link.MomentOfInertia;
        }
    }

    /// <summary>
    /// The force each pin exerts on its end A at this state, N. End B
    /// receives the opposite force. Includes the stabilization terms, which
    /// vanish on the constraint manifold.
    /// </summary>
    public void GetPinReactions(ReadOnlySpan<double> state, Span<Vector2d> reactions)
    {
        CheckLength(state.Length);
        if (reactions.Length != pins.Length)
        {
            throw new ArgumentException($"Expected {pins.Length} reactions, got {reactions.Length}.", nameof(reactions));
        }

        ComputeTotalForces(state, useLatchedModes: false);
        for (var p = 0; p < pins.Length; p++)
        {
            reactions[p] = new Vector2d(multipliers[2 * p], multipliers[(2 * p) + 1]);
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
    private void ComputeTotalForces(ReadOnlySpan<double> state, bool useLatchedModes)
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
