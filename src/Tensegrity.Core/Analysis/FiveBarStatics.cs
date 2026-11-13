using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Analysis;

/// <summary>
/// A five-bar configuration: the world angles of its four links. The
/// servo angle α is link A–C's (from A toward C), the passive angle β link
/// B–D's (from B toward D); the couplers' are from C toward E and from D
/// toward E. On the five-bar's configuration manifold both ways to E agree.
/// </summary>
public readonly record struct FiveBarPose(double Servo, double CouplerC, double Passive, double CouplerD)
{
    internal static FiveBarPose From(ReadOnlySpan<double> q) => new(q[0], q[1], q[2], q[3]);

    internal void CopyTo(Span<double> q)
    {
        q[0] = Servo;
        q[1] = CouplerC;
        q[2] = Passive;
        q[3] = CouplerD;
    }
}

/// <summary>Where an overdamped relaxation ended.</summary>
public enum FiveBarFlowEnd
{
    /// <summary>At a stationary point whose reduced Hessian is positive definite.</summary>
    Minimum,

    /// <summary>At a stationary point that is not a minimum (a start on a basin boundary).</summary>
    Saddle,

    /// <summary>Not at a stationary point within the iteration limit, or the step could not continue.</summary>
    Unconverged,
}

/// <summary>One overdamped relaxation.</summary>
/// <param name="Pose">Where it ended, angles normalized.</param>
/// <param name="End">What kind of point that is.</param>
/// <param name="LeftRange">True if the servo angle left the servo's range at any point on the way.</param>
/// <param name="Iterations">Steps taken.</param>
/// <param name="Residual">Largest component of the reduced gradient where it ended, N·m.</param>
public sealed record FiveBarRelaxation(FiveBarPose Pose, FiveBarFlowEnd End, bool LeftRange, int Iterations = 0, double Residual = 0);

/// <summary>A local minimum of a five-bar's potential.</summary>
/// <param name="Pose">The configuration, angles normalized.</param>
/// <param name="Side">+1 if E is left of the line from C to D, −1 if right.</param>
/// <param name="Energy">Potential energy, J (gravity measured from y = 0).</param>
/// <param name="SlowestFrequency">Lowest natural frequency of small oscillations about it, Hz, with the servo free.</param>
/// <param name="Tensions">Tendon tensions C, D, E and C–E, N.</param>
/// <param name="BasinStarts">How many grid starts inside the servo range relax to it.</param>
/// <param name="InRange">True if its servo angle is inside the servo's range.</param>
public sealed record FiveBarMinimum(
    FiveBarPose Pose,
    int Side,
    double Energy,
    double SlowestFrequency,
    IReadOnlyList<double> Tensions,
    int BasinStarts,
    bool InRange);

/// <summary>A jump of the passive coordinates while the servo angle was held: the tracked equilibrium vanished.</summary>
/// <param name="Servo">Servo angle at the step where it happened, rad.</param>
/// <param name="Jump">Change of the passive angle β across the jump, rad.</param>
public sealed record FiveBarSnap(double Servo, double Jump);

/// <summary>A quasi-static point on a transition: the servo held at an angle, the rest relaxed.</summary>
/// <param name="Servo">Servo angle α, rad.</param>
/// <param name="Passive">Passive angle β the shape relaxed to, rad.</param>
/// <param name="Torque">Torque the servo applies to link A–C to hold it there, N·m, counter-clockwise positive.</param>
public sealed record FiveBarPathPoint(double Servo, double Passive, double Torque);

/// <summary>
/// The servo driven quasi-statically from one minimum's angle to
/// another's, the passive coordinates relaxing at every step.
/// </summary>
/// <param name="From">Index of the starting minimum in <see cref="FiveBarReport.Minima"/>.</param>
/// <param name="To">Index of the target minimum.</param>
/// <param name="Arrived">True if the path ended at the target minimum.</param>
/// <param name="EndedAt">Index of the minimum the path ended at after releasing the servo there, or null if none.</param>
/// <param name="BlockedAt">Servo angle where no configuration could follow, rad; null if never blocked.</param>
/// <param name="PeakTorque">The path's largest servo torque by magnitude, N·m, with its sign.</param>
/// <param name="PeakAt">Servo angle of that peak, rad.</param>
/// <param name="Snaps">Jumps of the passive coordinates on the way.</param>
/// <param name="Path">Every step of the path.</param>
public sealed record FiveBarTransition(
    int From,
    int To,
    bool Arrived,
    int? EndedAt,
    double? BlockedAt,
    double PeakTorque,
    double PeakAt,
    IReadOnlyList<FiveBarSnap> Snaps,
    IReadOnlyList<FiveBarPathPoint> Path);

/// <summary>How a <see cref="FiveBarStatics"/> analysis searches and judges (the spec in brainstorm-multistable-servo.md).</summary>
/// <param name="GridStep">Spacing of the start grid in servo and passive angle, rad.</param>
/// <param name="PathStep">Servo angle step along a transition, rad.</param>
/// <param name="TorqueLimit">Largest servo torque allowed anywhere on a transition, N·m.</param>
/// <param name="MinimumGap">Smallest servo-angle gap allowed between neighbouring minima, rad.</param>
/// <param name="ServoMin">Low end of the servo's range, rad.</param>
/// <param name="ServoMax">High end of the servo's range, rad.</param>
/// <param name="SnapThreshold">A held step that moves β by more than this counts as a snap, rad.</param>
/// <param name="TargetCount">How many minima the spec asks for inside the range.</param>
public sealed record FiveBarSettings(
    double GridStep = 5 * Math.PI / 180,
    double PathStep = 0.25 * Math.PI / 180,
    double TorqueLimit = 3.4,
    double MinimumGap = 15 * Math.PI / 180,
    double ServoMin = 0,
    double ServoMax = Math.PI,
    double SnapThreshold = 1 * Math.PI / 180,
    int TargetCount = 4);

/// <summary>
/// A full statics analysis of one five-bar geometry against the spec.
/// </summary>
/// <param name="Settings">The settings it was judged with.</param>
/// <param name="Minima">Every distinct minimum reached from the grid, sorted by servo angle.</param>
/// <param name="Starts">Grid starts inside the servo range that assemble (both sides of E).</param>
/// <param name="LeftRange">Starts whose relaxation took the servo angle out of its range.</param>
/// <param name="Saddles">Starts that ended at a saddle.</param>
/// <param name="Unconverged">Starts that did not converge.</param>
/// <param name="Transitions">Each neighbouring pair of in-range minima, forward (rising servo angle) then back.</param>
/// <param name="Map">The grid of starts and where each ended.</param>
public sealed record FiveBarReport(
    FiveBarSettings Settings,
    IReadOnlyList<FiveBarMinimum> Minima,
    int Starts,
    int LeftRange,
    int Saddles,
    int Unconverged,
    IReadOnlyList<FiveBarTransition> Transitions,
    FiveBarBasinMap Map)
{
    /// <summary>The minima inside the servo range, in servo-angle order.</summary>
    public IReadOnlyList<FiveBarMinimum> InRange => Minima.Where(m => m.InRange).ToArray();

    /// <summary>Check 1: exactly the target number of minima inside the range.</summary>
    public bool CountOk => InRange.Count == Settings.TargetCount;

    /// <summary>Check 2: no minimum outside the range, and every start inside it relaxes, without leaving it, to a minimum inside it.</summary>
    public bool RangeOk => Minima.All(m => m.InRange) && LeftRange == 0 && Saddles == 0 && Unconverged == 0;

    /// <summary>Check 3: neighbouring minima inside the range at least the minimum gap apart in servo angle.</summary>
    public bool GapsOk
    {
        get
        {
            var inRange = InRange;
            for (var i = 1; i < inRange.Count; i++)
            {
                if (inRange[i].Pose.Servo - inRange[i - 1].Pose.Servo < Settings.MinimumGap)
                {
                    return false;
                }
            }

            return true;
        }
    }

    /// <summary>Check 4: every transition between neighbours, both ways, ends at its target.</summary>
    public bool TransitionsOk => Transitions.All(t => t.Arrived);

    /// <summary>Check 5: no transition needs more than the torque limit.</summary>
    public bool TorqueOk => Transitions.All(t => Math.Abs(t.PeakTorque) <= Settings.TorqueLimit);

    /// <summary>Largest servo torque magnitude over every transition, N·m (zero with none).</summary>
    public double PeakTorque => Transitions.Count == 0 ? 0 : Transitions.Max(t => Math.Abs(t.PeakTorque));

    /// <summary>All five statics checks pass. (Check 6, holding at the simulator's pin, is a test.)</summary>
    public bool Passes => CountOk && RangeOk && GapsOk && TransitionsOk && TorqueOk;
}

/// <summary>
/// Where each grid start ended. <see cref="Label"/> is the index of the
/// minimum in <see cref="FiveBarReport.Minima"/>, or
/// <see cref="NoAssembly"/> where the five-bar cannot close, or
/// <see cref="NoMinimum"/> for a saddle or an unconverged start.
/// </summary>
public sealed class FiveBarBasinMap
{
    public const int NoAssembly = -1;
    public const int NoMinimum = -2;

    private readonly int[] labels;
    private readonly bool[] leftRange;

    internal FiveBarBasinMap(IReadOnlyList<double> servo, IReadOnlyList<double> passive, int[] labels, bool[] leftRange)
    {
        Servo = servo;
        Passive = passive;
        this.labels = labels;
        this.leftRange = leftRange;
    }

    /// <summary>Servo angles of the grid rows, rad.</summary>
    public IReadOnlyList<double> Servo { get; }

    /// <summary>Passive angles of the grid columns, rad.</summary>
    public IReadOnlyList<double> Passive { get; }

    /// <summary>Where the start at row <paramref name="i"/>, column <paramref name="j"/>, with E on <paramref name="side"/> (+1 or −1), ended.</summary>
    public int Label(int side, int i, int j) => labels[Index(side, i, j)];

    /// <summary>Whether that start's relaxation left the servo range.</summary>
    public bool Left(int side, int i, int j) => leftRange[Index(side, i, j)];

    internal int Index(int side, int i, int j) => (((side > 0 ? 0 : 1) * Servo.Count) + i) * Passive.Count + j;
}

/// <summary>
/// The statics of a <see cref="FiveBarGeometry"/>, written independently of
/// the simulator, as <c>ChainStatics</c> is for the wing.
///
/// Coordinates q = (α, θ_C, β, θ_D): the four links' world angles, with two
/// closure constraints g(q) = A + a·u(α) + c·u(θ_C) − B − b·u(β) − d·u(θ_D) = 0,
/// where u(θ) = (cos θ, sin θ). E is reached through A–C–E. The potential is
/// <code>
/// V(q) = g·Σ m·y_cm + Σ ½·k·(L − L0)²   (stretched tendons only, k = EA / L0)
/// </code>
/// with an analytic gradient. Its Hessian is a central difference of the
/// gradient, and on the manifold it gains the constraints' curvature,
/// Σ λ·∂²g with λ the multipliers.
///
/// Relaxing a start follows the overdamped flow M·q̇ = −∇V on the manifold
/// (M the mass matrix), the slow-motion limit of the simulator with heavy
/// global damping. Each step is linearly implicit Euler in the manifold's
/// tangent space, (s·M_r + H_r)·δ = −g_r, so stiff tendon modes do not
/// limit the step. s shrinks as steps succeed, which turns the steps into
/// Newton's near a minimum. Each step is capped at <see cref="MaxStep"/>
/// so the flow does not jump basins, and is then projected back on to the
/// constraints.
/// </summary>
public sealed class FiveBarStatics
{
    /// <summary>Largest change of any angle in one relaxation step, rad.</summary>
    public const double MaxStep = 2 * Math.PI / 180;

    private const int N = 4;
    private const int Servo = 0;
    private const int MaxIterations = 2000;
    private const double GradientTolerance = 1e-11;
    private const double ProjectionTolerance = 1e-13;
    private const double SameMinimum = 1e-5;

    // +1 for the A–C–E chain, −1 for B–D–E, in g(q).
    private static readonly double[] Sign = [1, 1, -1, -1];

    private readonly double[] length;
    private readonly Vector2d a;
    private readonly Vector2d b;
    private readonly FiveBarTendon[] tendons;

    // How far along the last link of its chain each tendon attaches, as a fraction of that link.
    private readonly double[] fraction;

    public FiveBarStatics(FiveBarGeometry geometry)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        geometry.Validate();
        Geometry = geometry;
        length = [geometry.ServoLength, geometry.CouplerCLength, geometry.PassiveLength, geometry.CouplerDLength];
        a = Vector2d.Zero;
        b = new Vector2d(geometry.BaseWidth, 0);
        tendons = [geometry.TendonC, geometry.TendonD, geometry.TendonE, geometry.TendonCE];
        fraction = [1, 1, 1, geometry.TendonCEPosition];
    }

    public FiveBarGeometry Geometry { get; }

    public Vector2d PointA => a;

    public Vector2d PointB => b;

    public Vector2d PointC(FiveBarPose p) => a + (length[0] * U(p.Servo));

    public Vector2d PointD(FiveBarPose p) => b + (length[2] * U(p.Passive));

    /// <summary>E through A–C–E.</summary>
    public Vector2d PointE(FiveBarPose p) => PointC(p) + (length[1] * U(p.CouplerC));

    /// <summary>
    /// Assembles the five-bar with link A–C at <paramref name="servo"/> and
    /// B–D at <paramref name="passive"/>, E on <paramref name="side"/> of the
    /// line from C to D (+1 left, −1 right). False if the couplers cannot
    /// reach.
    /// </summary>
    public bool TryAssemble(double servo, double passive, int side, out FiveBarPose pose)
    {
        var c = a + (length[0] * U(servo));
        var d = b + (length[2] * U(passive));
        var cd = d - c;
        var dist = cd.Length;
        var lc = length[1];
        var ld = length[3];
        pose = default;
        if (!(dist > 0) || dist > lc + ld || dist < Math.Abs(lc - ld))
        {
            return false;
        }

        var along = ((lc * lc) - (ld * ld) + (dist * dist)) / (2 * dist);
        var across = Math.Sqrt(Math.Max(0, (lc * lc) - (along * along)));
        var unit = cd / dist;
        var e = c + (along * unit) + ((side >= 0 ? across : -across) * unit.Perpendicular);
        pose = new FiveBarPose(servo, Angle(e - c), passive, Angle(e - d));
        return true;
    }

    /// <summary>+1 if E is left of the line from C to D, −1 if right.</summary>
    public int Side(FiveBarPose p) => Vector2d.Cross(PointD(p) - PointC(p), PointE(p) - PointC(p)) >= 0 ? 1 : -1;

    /// <summary>Potential energy V, J: gravity (from y = 0) plus stretched tendons.</summary>
    public double Energy(FiveBarPose p)
    {
        Span<double> q = stackalloc double[N];
        p.CopyTo(q);
        return Energy(q);
    }

    /// <summary>Tendon tensions C, D, E and C–E, N.</summary>
    public double[] Tensions(FiveBarPose p)
    {
        Span<double> q = stackalloc double[N];
        p.CopyTo(q);
        var result = new double[tendons.Length];
        for (var t = 0; t < tendons.Length; t++)
        {
            var (point, _, _) = TendonPoint(q, t);
            result[t] = Tension(t, (point - tendons[t].Anchor).Length);
        }

        return result;
    }

    /// <summary>
    /// The torque the servo must apply to link A–C to hold <paramref name="p"/>,
    /// N·m (counter-clockwise positive), when every other coordinate is in
    /// equilibrium there: τ from ∇V = τ·e_α + Gᵀ·μ.
    /// </summary>
    public double ServoTorque(FiveBarPose p)
    {
        Span<double> q = stackalloc double[N];
        p.CopyTo(q);
        Span<double> grad = stackalloc double[N];
        Gradient(q, grad);
        Span<double> rows = stackalloc double[3 * N];
        ConstraintRows(q, fixServo: true, rows);
        Span<double> multipliers = stackalloc double[3];
        LeastSquaresMultipliers(rows, 3, grad, multipliers);
        return multipliers[2];
    }

    /// <summary>Lowest natural frequency of small oscillations about <paramref name="p"/>, Hz; NaN if it is not a minimum.</summary>
    public double SlowestFrequency(FiveBarPose p)
    {
        Span<double> q = stackalloc double[N];
        p.CopyTo(q);
        return Classify(q, fixServo: false, out var lowest) ? Math.Sqrt(lowest) / (2 * Math.PI) : double.NaN;
    }

    /// <summary>Relaxes <paramref name="start"/> along the overdamped flow with the servo free.</summary>
    public FiveBarRelaxation Relax(FiveBarPose start, double servoMin = double.NegativeInfinity, double servoMax = double.PositiveInfinity)
    {
        Span<double> q = stackalloc double[N];
        start.CopyTo(q);
        var end = Descend(q, fixServo: false, servoMin, servoMax, out var left, out var iterations, out var residual);
        return new FiveBarRelaxation(Normalize(FiveBarPose.From(q)), end, left, iterations, residual);
    }

    /// <summary>Relaxes <paramref name="start"/> with the servo held at its angle.</summary>
    public FiveBarRelaxation RelaxHeld(FiveBarPose start)
    {
        Span<double> q = stackalloc double[N];
        start.CopyTo(q);
        var end = Descend(q, fixServo: true, double.NegativeInfinity, double.PositiveInfinity, out _, out var iterations, out var residual);
        return new FiveBarRelaxation(FiveBarPose.From(q), end, false, iterations, residual);
    }

    /// <summary>Finds every minimum reachable from the start grid and judges the geometry against the spec.</summary>
    public FiveBarReport Analyze(FiveBarSettings? settings = null)
    {
        var s = settings ?? new FiveBarSettings();
        if (!(s.GridStep > 0) || !(s.PathStep > 0) || !(s.ServoMax > s.ServoMin))
        {
            throw new ArgumentException("The grid and path steps must be positive and the servo range not empty.", nameof(settings));
        }

        var rows = (int)Math.Floor(((s.ServoMax - s.ServoMin) / s.GridStep) + 1e-9) + 1;
        var columns = (int)Math.Round(2 * Math.PI / s.GridStep);
        var servo = Enumerable.Range(0, rows).Select(i => s.ServoMin + (i * s.GridStep)).ToArray();
        var passive = Enumerable.Range(0, columns).Select(j => (j * 2 * Math.PI / columns) - Math.PI).ToArray();
        var count = 2 * rows * columns;
        var results = new FiveBarRelaxation?[count];
        Parallel.For(0, count, k =>
        {
            var side = k < rows * columns ? 1 : -1;
            var rest = k % (rows * columns);
            if (TryAssemble(servo[rest / columns], passive[rest % columns], side, out var pose))
            {
                results[k] = Relax(pose, s.ServoMin, s.ServoMax);
            }
        });

        // Cluster in grid order, so the result does not depend on threading.
        var found = new List<FiveBarPose>();
        var starts = new int[count];
        var labels = new int[count];
        var left = new bool[count];
        int leftRange = 0, saddles = 0, unconverged = 0, assembled = 0;
        for (var k = 0; k < count; k++)
        {
            var r = results[k];
            if (r is null)
            {
                labels[k] = FiveBarBasinMap.NoAssembly;
                continue;
            }

            assembled++;
            left[k] = r.LeftRange;
            leftRange += r.LeftRange ? 1 : 0;
            if (r.End != FiveBarFlowEnd.Minimum)
            {
                labels[k] = FiveBarBasinMap.NoMinimum;
                saddles += r.End == FiveBarFlowEnd.Saddle ? 1 : 0;
                unconverged += r.End == FiveBarFlowEnd.Unconverged ? 1 : 0;
                continue;
            }

            var index = found.FindIndex(m => Distance(m, r.Pose) < SameMinimum);
            if (index < 0)
            {
                index = found.Count;
                found.Add(r.Pose);
            }

            labels[k] = index;
            starts[index]++;
        }

        // Sort by servo angle and relabel.
        var order = Enumerable.Range(0, found.Count).OrderBy(i => found[i].Servo).ThenBy(i => found[i].Passive).ToArray();
        var rank = new int[found.Count];
        for (var i = 0; i < order.Length; i++)
        {
            rank[order[i]] = i;
        }

        for (var k = 0; k < count; k++)
        {
            labels[k] = labels[k] >= 0 ? rank[labels[k]] : labels[k];
        }

        var minima = order.Select(i => new FiveBarMinimum(
            found[i],
            Side(found[i]),
            Energy(found[i]),
            SlowestFrequency(found[i]),
            Tensions(found[i]),
            starts[i],
            found[i].Servo >= s.ServoMin && found[i].Servo <= s.ServoMax)).ToArray();

        var inRange = Enumerable.Range(0, minima.Length).Where(i => minima[i].InRange).ToArray();
        var pairs = new List<(int From, int To)>();
        for (var i = 1; i < inRange.Length; i++)
        {
            pairs.Add((inRange[i - 1], inRange[i]));
        }

        foreach (var (from, to) in pairs.ToArray())
        {
            pairs.Add((to, from));
        }

        var transitions = new FiveBarTransition[pairs.Count];
        Parallel.For(0, pairs.Count, i => transitions[i] = Transition(minima, pairs[i].From, pairs[i].To, s));

        return new FiveBarReport(s, minima, assembled, leftRange, saddles, unconverged, transitions, new FiveBarBasinMap(servo, passive, labels, left));
    }

    /// <summary>
    /// Drives the servo quasi-statically from minimum <paramref name="from"/>
    /// to minimum <paramref name="to"/>'s angle in <see cref="FiveBarSettings.PathStep"/>
    /// steps, relaxing the rest at each, then releases it there.
    /// </summary>
    public FiveBarTransition Transition(IReadOnlyList<FiveBarMinimum> minima, int from, int to, FiveBarSettings settings)
    {
        ArgumentNullException.ThrowIfNull(minima);
        ArgumentNullException.ThrowIfNull(settings);
        var start = minima[from].Pose;
        var target = minima[to].Pose.Servo;
        var steps = Math.Max(1, (int)Math.Ceiling(Math.Abs(target - start.Servo) / settings.PathStep));
        Span<double> q = stackalloc double[N];
        start.CopyTo(q);
        var path = new List<FiveBarPathPoint> { new(start.Servo, start.Passive, ServoTorque(start)) };
        var snaps = new List<FiveBarSnap>();
        double? blocked = null;
        for (var k = 1; k <= steps; k++)
        {
            var servo = start.Servo + ((target - start.Servo) * k / steps);
            q[Servo] = servo;
            if (!Project(q, fixServo: true))
            {
                blocked = servo;
                break;
            }

            var before = q[2];
            if (Descend(q, fixServo: true, double.NegativeInfinity, double.PositiveInfinity, out _, out _, out _) == FiveBarFlowEnd.Unconverged)
            {
                blocked = servo;
                break;
            }

            var jump = Wrap(q[2] - before);
            if (Math.Abs(jump) > settings.SnapThreshold)
            {
                snaps.Add(new FiveBarSnap(servo, jump));
            }

            var pose = FiveBarPose.From(q);
            path.Add(new FiveBarPathPoint(servo, pose.Passive, ServoTorque(pose)));
        }

        var peak = path.MaxBy(p => Math.Abs(p.Torque))!;
        int? endedAt = null;
        var arrived = false;
        if (blocked is null)
        {
            var released = Relax(FiveBarPose.From(q));
            if (released.End == FiveBarFlowEnd.Minimum)
            {
                var index = Enumerable.Range(0, minima.Count).FirstOrDefault(i => Distance(minima[i].Pose, released.Pose) < 1e-4, -1);
                endedAt = index >= 0 ? index : null;
                arrived = index == to;
            }
        }

        return new FiveBarTransition(from, to, arrived, endedAt, blocked, peak.Torque, peak.Servo, snaps, path);
    }

    /// <summary>Largest wrapped difference between two poses' angles, rad.</summary>
    public static double Distance(FiveBarPose x, FiveBarPose y) =>
        Math.Max(
            Math.Max(Math.Abs(Wrap(x.Servo - y.Servo)), Math.Abs(Wrap(x.CouplerC - y.CouplerC))),
            Math.Max(Math.Abs(Wrap(x.Passive - y.Passive)), Math.Abs(Wrap(x.CouplerD - y.CouplerD))));

    /// <summary>The servo angle in (−π/2, 3π/2], so its range 0 to π is one interval; the rest in (−π, π].</summary>
    public static FiveBarPose Normalize(FiveBarPose p)
    {
        var servo = Wrap(p.Servo - (Math.PI / 2)) + (Math.PI / 2);
        return new FiveBarPose(servo, Wrap(p.CouplerC), Wrap(p.Passive), Wrap(p.CouplerD));
    }

    private static double Wrap(double angle) => angle - (2 * Math.PI * Math.Ceiling((angle - Math.PI) / (2 * Math.PI)));

    private static Vector2d U(double t) => new(Math.Cos(t), Math.Sin(t));

    private static Vector2d Perp(double t) => new(-Math.Sin(t), Math.Cos(t));

    private static double Angle(Vector2d v) => Math.Atan2(v.Y, v.X);

    private double Tension(int t, double distance) =>
        distance > tendons[t].RestLength ? Geometry.AxialStiffness / tendons[t].RestLength * (distance - tendons[t].RestLength) : 0;

    // Chains: links 0 and 1 hang from A, links 2 and 3 from B.
    private static int ChainStart(int link) => link < 2 ? 0 : 2;

    private Vector2d ChainOrigin(int link) => link < 2 ? a : b;

    /// <summary>
    /// The tendon's attachment and the range of coordinates [first, last]
    /// that move it, each by <see cref="Reach"/>.
    /// </summary>
    private (Vector2d Point, int First, int Last) TendonPoint(ReadOnlySpan<double> q, int t)
    {
        var (first, last) = t switch
        {
            0 => (0, 0), // C
            1 => (2, 2), // D
            _ => (0, 1), // E, and the point on C–E, through C
        };
        var point = ChainOrigin(first);
        for (var j = first; j <= last; j++)
        {
            point += Reach(t, j, last) * U(q[j]);
        }

        return (point, first, last);
    }

    /// <summary>How far along link j tendon t's attachment lies: the whole link inboard, the attachment's fraction of the last.</summary>
    private double Reach(int t, int j, int last) => j < last ? length[j] : fraction[t] * length[j];

    private Vector2d CenterOfMass(ReadOnlySpan<double> q, int link)
    {
        var point = ChainOrigin(link);
        for (var j = ChainStart(link); j < link; j++)
        {
            point += length[j] * U(q[j]);
        }

        return point + (length[link] / 2 * U(q[link]));
    }

    private double Energy(ReadOnlySpan<double> q)
    {
        var v = 0.0;
        for (var k = 0; k < N; k++)
        {
            v += Geometry.LinkMass * Geometry.Gravity * CenterOfMass(q, k).Y;
        }

        for (var t = 0; t < tendons.Length; t++)
        {
            var (point, _, _) = TendonPoint(q, t);
            var stretch = (point - tendons[t].Anchor).Length - tendons[t].RestLength;
            if (stretch > 0)
            {
                v += 0.5 * Geometry.AxialStiffness / tendons[t].RestLength * stretch * stretch;
            }
        }

        return v;
    }

    private void Gradient(ReadOnlySpan<double> q, Span<double> grad)
    {
        grad.Clear();
        var weight = Geometry.LinkMass * Geometry.Gravity;
        for (var k = 0; k < N; k++)
        {
            // ∂y_cm/∂q_j = r_j·cos q_j, with r_j the full length inboard and half the link's own.
            for (var j = ChainStart(k); j <= k; j++)
            {
                grad[j] += weight * (j < k ? length[j] : length[k] / 2) * Math.Cos(q[j]);
            }
        }

        for (var t = 0; t < tendons.Length; t++)
        {
            var (point, first, last) = TendonPoint(q, t);
            var r = point - tendons[t].Anchor;
            var distance = r.Length;
            var tension = Tension(t, distance);
            if (tension > 0)
            {
                var unit = r / distance;
                for (var j = first; j <= last; j++)
                {
                    grad[j] += tension * Vector2d.Dot(unit, Reach(t, j, last) * Perp(q[j]));
                }
            }
        }
    }

    private void MassMatrix(ReadOnlySpan<double> q, Span<double> m)
    {
        m.Clear();
        var mass = Geometry.LinkMass;
        for (var k = 0; k < N; k++)
        {
            var start = ChainStart(k);
            for (var i = start; i <= k; i++)
            {
                var ji = (i < k ? length[i] : length[k] / 2) * Perp(q[i]);
                for (var j = start; j <= k; j++)
                {
                    var jj = (j < k ? length[j] : length[k] / 2) * Perp(q[j]);
                    m[(i * N) + j] += mass * Vector2d.Dot(ji, jj);
                }
            }

            m[(k * N) + k] += mass * length[k] * length[k] / 12;
        }
    }

    private Vector2d Closure(ReadOnlySpan<double> q)
    {
        var g = a - b;
        for (var j = 0; j < N; j++)
        {
            g += Sign[j] * length[j] * U(q[j]);
        }

        return g;
    }

    /// <summary>Rows of the constraint Jacobian: ∂g_x, ∂g_y, and e_α when the servo is held.</summary>
    private void ConstraintRows(ReadOnlySpan<double> q, bool fixServo, Span<double> rows)
    {
        rows.Clear();
        for (var j = 0; j < N; j++)
        {
            var column = Sign[j] * length[j] * Perp(q[j]);
            rows[j] = column.X;
            rows[N + j] = column.Y;
        }

        if (fixServo)
        {
            rows[(2 * N) + Servo] = 1;
        }
    }

    /// <summary>Multipliers μ minimizing |Rᵀ·μ − grad| for the <paramref name="count"/> rows R.</summary>
    private static void LeastSquaresMultipliers(ReadOnlySpan<double> rows, int count, ReadOnlySpan<double> grad, Span<double> multipliers)
    {
        Span<double> normal = stackalloc double[count * count];
        for (var r = 0; r < count; r++)
        {
            multipliers[r] = 0;
            for (var j = 0; j < N; j++)
            {
                multipliers[r] += rows[(r * N) + j] * grad[j];
            }

            for (var c = 0; c <= r; c++)
            {
                var sum = 0.0;
                for (var j = 0; j < N; j++)
                {
                    sum += rows[(r * N) + j] * rows[(c * N) + j];
                }

                normal[(r * count) + c] = sum;
            }
        }

        if (!DenseCholesky.TryFactor(normal, count))
        {
            multipliers.Clear();
            return;
        }

        DenseCholesky.Solve(normal, count, multipliers);
    }

    /// <summary>Hessian of V on the manifold: central differences of ∇V, plus the constraint curvature.</summary>
    private void Hessian(ReadOnlySpan<double> q, ReadOnlySpan<double> grad, ReadOnlySpan<double> rows, int rowCount, Span<double> h)
    {
        const double step = 1e-6;
        Span<double> shifted = stackalloc double[N];
        Span<double> plus = stackalloc double[N];
        Span<double> minus = stackalloc double[N];
        for (var j = 0; j < N; j++)
        {
            q.CopyTo(shifted);
            shifted[j] = q[j] + step;
            Gradient(shifted, plus);
            shifted[j] = q[j] - step;
            Gradient(shifted, minus);
            for (var i = 0; i < N; i++)
            {
                h[(i * N) + j] = (plus[i] - minus[i]) / (2 * step);
            }
        }

        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j < i; j++)
            {
                var mean = 0.5 * (h[(i * N) + j] + h[(j * N) + i]);
                h[(i * N) + j] = mean;
                h[(j * N) + i] = mean;
            }
        }

        // ∇V = Gᵀλ (+ τ·e_α): the closure's curvature −λ·∂²g = λ·sign·r·u(q_j) on the diagonal.
        Span<double> multipliers = stackalloc double[rowCount];
        LeastSquaresMultipliers(rows, rowCount, grad, multipliers);
        var lambda = new Vector2d(multipliers[0], multipliers[1]);
        for (var j = 0; j < N; j++)
        {
            h[(j * N) + j] += Sign[j] * length[j] * Vector2d.Dot(lambda, U(q[j]));
        }
    }

    /// <summary>An orthonormal basis of the tangent space (the rows' null space) as columns of <paramref name="z"/> (N × free).</summary>
    private static void TangentBasis(ReadOnlySpan<double> rows, int rowCount, Span<double> z)
    {
        Span<double> gram = stackalloc double[N * N];
        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                var sum = 0.0;
                for (var r = 0; r < rowCount; r++)
                {
                    sum += rows[(r * N) + i] * rows[(r * N) + j];
                }

                gram[(i * N) + j] = sum;
            }
        }

        Span<double> values = stackalloc double[N];
        Span<double> vectors = stackalloc double[N * N];
        SymmetricEigen.Decompose(gram, N, values, vectors);
        Span<int> order = stackalloc int[N];
        for (var i = 0; i < N; i++)
        {
            order[i] = i;
        }

        // Smallest eigenvalues first (insertion sort, ties by index).
        for (var i = 1; i < N; i++)
        {
            for (var k = i; k > 0 && values[order[k]] < values[order[k - 1]]; k--)
            {
                (order[k], order[k - 1]) = (order[k - 1], order[k]);
            }
        }

        var free = N - rowCount;
        for (var c = 0; c < free; c++)
        {
            for (var i = 0; i < N; i++)
            {
                z[(i * free) + c] = vectors[(i * N) + order[c]];
            }
        }
    }

    /// <summary>Zᵀ·A·Z for a symmetric N × N matrix A.</summary>
    private static void Reduce(ReadOnlySpan<double> matrix, ReadOnlySpan<double> z, int free, Span<double> reduced)
    {
        for (var r = 0; r < free; r++)
        {
            for (var c = 0; c < free; c++)
            {
                var sum = 0.0;
                for (var i = 0; i < N; i++)
                {
                    for (var j = 0; j < N; j++)
                    {
                        sum += z[(i * free) + r] * matrix[(i * N) + j] * z[(j * free) + c];
                    }
                }

                reduced[(r * free) + c] = sum;
            }
        }
    }

    /// <summary>Restores the closure by minimum-norm corrections to the free coordinates (all but α when the servo is held).</summary>
    private bool Project(Span<double> q, bool fixServo)
    {
        var first = fixServo ? 1 : 0;
        for (var iteration = 0; iteration < 50; iteration++)
        {
            var g = Closure(q);
            if (g.Length < ProjectionTolerance)
            {
                return true;
            }

            // G_s·G_sᵀ over the free columns, 2 × 2.
            double gxx = 0, gxy = 0, gyy = 0;
            for (var j = first; j < N; j++)
            {
                var column = Sign[j] * length[j] * Perp(q[j]);
                gxx += column.X * column.X;
                gxy += column.X * column.Y;
                gyy += column.Y * column.Y;
            }

            var det = (gxx * gyy) - (gxy * gxy);
            if (!(Math.Abs(det) > 1e-14 * ((gxx * gyy) + 1e-300)))
            {
                return false;
            }

            var mx = ((gyy * g.X) - (gxy * g.Y)) / det;
            var my = ((gxx * g.Y) - (gxy * g.X)) / det;
            for (var j = first; j < N; j++)
            {
                var column = Sign[j] * length[j] * Perp(q[j]);
                q[j] -= (column.X * mx) + (column.Y * my);
            }
        }

        return Closure(q).Length < 1e-10;
    }

    /// <summary>
    /// Reduced Hessian test at a stationary point: true if positive
    /// definite, with <paramref name="lowest"/> the smallest eigenvalue of
    /// M_r⁻¹·H_r (ω², rad²/s²).
    /// </summary>
    private bool Classify(ReadOnlySpan<double> q, bool fixServo, out double lowest)
    {
        var rowCount = fixServo ? 3 : 2;
        var free = N - rowCount;
        Span<double> grad = stackalloc double[N];
        Span<double> rows = stackalloc double[3 * N];
        Span<double> z = stackalloc double[N * 2];
        Span<double> h = stackalloc double[N * N];
        Span<double> m = stackalloc double[N * N];
        Span<double> hr = stackalloc double[4];
        Span<double> mr = stackalloc double[4];
        Gradient(q, grad);
        ConstraintRows(q, fixServo, rows);
        TangentBasis(rows, rowCount, z);
        Hessian(q, grad, rows, rowCount, h);
        MassMatrix(q, m);
        Reduce(h, z, free, hr);
        Reduce(m, z, free, mr);
        if (free == 1)
        {
            lowest = hr[0] / mr[0];
        }
        else
        {
            // det(H − λM) = 0 for 2 × 2: (m00 m11 − m01²) λ² − (h00 m11 + h11 m00 − 2 h01 m01) λ + (h00 h11 − h01²) = 0.
            var qa = (mr[0] * mr[3]) - (mr[1] * mr[1]);
            var qb = -((hr[0] * mr[3]) + (hr[3] * mr[0]) - (2 * hr[1] * mr[1]));
            var qc = (hr[0] * hr[3]) - (hr[1] * hr[1]);
            var disc = Math.Sqrt(Math.Max(0, (qb * qb) - (4 * qa * qc)));
            lowest = (-qb - disc) / (2 * qa);
        }

        var scale = Math.Abs(hr[0]) + Math.Abs(hr[free * free - 1]);
        return lowest > 1e-9 * scale / mr[0];
    }

    /// <summary>Largest component of the reduced gradient Zᵀ·∇V at q.</summary>
    private double ReducedResidual(ReadOnlySpan<double> q, bool fixServo)
    {
        var rowCount = fixServo ? 3 : 2;
        var free = N - rowCount;
        Span<double> grad = stackalloc double[N];
        Span<double> rows = stackalloc double[3 * N];
        Span<double> z = stackalloc double[N * 2];
        Gradient(q, grad);
        ConstraintRows(q, fixServo, rows);
        TangentBasis(rows, rowCount, z);
        var largest = 0.0;
        for (var c = 0; c < free; c++)
        {
            var sum = 0.0;
            for (var i = 0; i < N; i++)
            {
                sum += z[(i * free) + c] * grad[i];
            }

            largest = Math.Max(largest, Math.Abs(sum));
        }

        return largest;
    }

    /// <summary>The overdamped flow from q to a stationary point, by linearly implicit Euler steps on the manifold.</summary>
    private FiveBarFlowEnd Descend(Span<double> q, bool fixServo, double servoMin, double servoMax, out bool leftRange, out int iterations, out double residual)
    {
        leftRange = false;
        iterations = 0;
        residual = double.NaN;
        if (!Project(q, fixServo))
        {
            return FiveBarFlowEnd.Unconverged;
        }

        var rowCount = fixServo ? 3 : 2;
        var free = N - rowCount;
        Span<double> grad = stackalloc double[N];
        Span<double> rows = stackalloc double[3 * N];
        Span<double> z = stackalloc double[N * 2];
        Span<double> h = stackalloc double[N * N];
        Span<double> m = stackalloc double[N * N];
        Span<double> hr = stackalloc double[4];
        Span<double> mr = stackalloc double[4];
        Span<double> gr = stackalloc double[2];
        Span<double> system = stackalloc double[4];
        Span<double> delta = stackalloc double[2];
        Span<double> trial = stackalloc double[N];
        var shift = 1e6;
        var energy = Energy(q);
        for (var iteration = 0; iteration < MaxIterations; iteration++)
        {
            iterations = iteration;
            Gradient(q, grad);
            ConstraintRows(q, fixServo, rows);
            TangentBasis(rows, rowCount, z);
            var largest = 0.0;
            for (var c = 0; c < free; c++)
            {
                gr[c] = 0;
                for (var i = 0; i < N; i++)
                {
                    gr[c] += z[(i * free) + c] * grad[i];
                }

                largest = Math.Max(largest, Math.Abs(gr[c]));
            }

            residual = largest;

            // Relative to the force level: at a minimum the full gradient is the closure's reaction, Gᵀλ.
            var forceLevel = 1.0;
            for (var i = 0; i < N; i++)
            {
                forceLevel = Math.Max(forceLevel, Math.Abs(grad[i]));
            }

            if (largest < GradientTolerance * forceLevel)
            {
                return Classify(q, fixServo, out _) ? FiveBarFlowEnd.Minimum : FiveBarFlowEnd.Saddle;
            }

            Hessian(q, grad, rows, rowCount, h);
            MassMatrix(q, m);
            Reduce(h, z, free, hr);
            Reduce(m, z, free, mr);

            var accepted = false;
            while (!accepted)
            {
                if (shift > 1e30)
                {
                    // The step cannot decrease V any more: at a stationary point to round-off, or stuck.
                    return largest < 1e-6
                        ? Classify(q, fixServo, out _) ? FiveBarFlowEnd.Minimum : FiveBarFlowEnd.Saddle
                        : FiveBarFlowEnd.Unconverged;
                }

                for (var i = 0; i < free * free; i++)
                {
                    system[i] = (shift * mr[i]) + hr[i];
                }

                if (!DenseCholesky.TryFactor(system[..(free * free)], free))
                {
                    shift *= 4;
                    continue;
                }

                for (var c = 0; c < free; c++)
                {
                    delta[c] = -gr[c];
                }

                DenseCholesky.Solve(system[..(free * free)], free, delta[..free]);
                var biggest = 0.0;
                for (var i = 0; i < N; i++)
                {
                    var dq = 0.0;
                    for (var c = 0; c < free; c++)
                    {
                        dq += z[(i * free) + c] * delta[c];
                    }

                    biggest = Math.Max(biggest, Math.Abs(dq));
                }

                var scale = biggest > MaxStep ? MaxStep / biggest : 1.0;
                var decrease = 0.0;
                for (var c = 0; c < free; c++)
                {
                    decrease -= scale * gr[c] * delta[c];
                }

                q.CopyTo(trial);
                for (var i = 0; i < N; i++)
                {
                    for (var c = 0; c < free; c++)
                    {
                        trial[i] += scale * z[(i * free) + c] * delta[c];
                    }
                }

                if (!Project(trial, fixServo))
                {
                    shift *= 4;
                    continue;
                }

                // Sufficient decrease; or, once the decrease is below the energy's round-off, a smaller gradient.
                var trialEnergy = Energy(trial);
                var roundOff = 1e-12 * (Math.Abs(energy) + 1);
                if (!(trialEnergy <= energy - (1e-4 * decrease))
                    && !(trialEnergy <= energy + roundOff && ReducedResidual(trial, fixServo) < largest))
                {
                    shift *= 4;
                    continue;
                }

                trial.CopyTo(q);
                energy = trialEnergy;
                shift = Math.Max(shift / 4, 1e-12);
                accepted = true;
            }

            if (!fixServo && (q[Servo] < servoMin || q[Servo] > servoMax))
            {
                leftRange = true;
            }
        }

        return FiveBarFlowEnd.Unconverged;
    }
}
