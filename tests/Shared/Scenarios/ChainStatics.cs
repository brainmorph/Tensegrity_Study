using Tensegrity.Core.Mathematics;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>A lock threshold of one tendon: the rest-length scale where the stable pose vanishes, and the pose there (world link angles).</summary>
public sealed record ChainFold(int Tendon, double Scale, IReadOnlyList<double> Angles);

/// <summary>
/// The energy barrier a snap latch crosses at one joint: the joint angle of
/// the saddle, the potential there, and the pose (world link angles).
/// </summary>
public sealed record ChainBarrier(int Joint, double JointAngle, double Potential, IReadOnlyList<double> Angles);

/// <summary>
/// The geometric prediction for test 10 (PLAN.md section 7), generalized
/// from <see cref="ArmStatics"/> to an open chain of n links pinned end to
/// end from a shoulder at the origin, one tendon per joint, written
/// independently of the simulator. With θ the links' world angles,
/// <code>
/// V(θ) = g·Σ mᵢ·yᵢ + Σ ½·k·(L − s·L0)²   (stretched tendons only)
/// </code>
/// and k = EA / (s·L0), plus ½·κⱼ·(φⱼ − uⱼ)² for each joint j pressed past
/// a hyperextension stop at uⱼ (φⱼ = θⱼ − θⱼ₋₁, κⱼ its stiffness), and
/// ½·κⱼ·(lⱼ − φⱼ)² for one folded past a fold stop at lⱼ. Tendon j runs from an anchor behind and above joint j
/// (on the ground for the shoulder, else on link j − 1) to a point on link j,
/// on its axis or above it.
/// Turning link k alone about its inner joint Jₖ (its world angle θₖ) moves a
/// point P of link k as ⊥(P − Jₖ) and carries every outer link along by
/// ⊥(Jₖ₊₁ − Jₖ), which gives the gradient exactly; the Hessian is a central
/// difference of it.
///
/// Rest poses are zeros of ∇V, stable (locked) where the Hessian is
/// positive definite. Lengthening tendon j moves the stable pose along a
/// branch; driving link j's angle down the branch, the scale rises to a
/// maximum and falls again on the unstable side. That maximum is the fold:
/// past it no stable pose is left and the chain collapses.
/// </summary>
public sealed class ChainStatics
{
    private readonly Segment[] links;
    private readonly Tendon[] tendons;
    private readonly Stop?[] stops;
    private readonly double axialStiffness;
    private readonly double gravity;

    private ChainStatics(Segment[] links, Tendon[] tendons, double axialStiffness, double gravity, Stop?[]? stops = null)
    {
        this.links = links;
        this.tendons = tendons;
        this.stops = stops ?? new Stop?[links.Length];
        this.axialStiffness = axialStiffness;
        this.gravity = gravity;
    }

    public int LinkCount => links.Length;

    /// <summary>Statics of a two-link <see cref="LockingArm"/>, with the rest lengths its model carries at scale 1.</summary>
    public static ChainStatics ForArm(PlanarModel scaleOne, LockingArmGeometry? geometry = null)
    {
        var g = geometry ?? new LockingArmGeometry();
        var c = scaleOne.System.Cables;
        return new ChainStatics(
            [new Segment(g.UpperLength, g.UpperMass), new Segment(g.LowerLength, g.LowerMass)],
            [
                new Tendon(g.ShoulderAnchorBehind, g.ShoulderAnchorAbove, g.ShoulderInsertion, c[0].RestLength),
                new Tendon(g.ElbowPostBehind, g.ElbowPostAbove, g.ElbowInsertion, c[1].RestLength),
            ],
            g.AxialStiffness,
            9.81);
    }

    /// <summary>
    /// Statics of a three-link <see cref="LockingWing"/> designed straight
    /// out, with the rest lengths its model carries at scale 1. Hyperextension
    /// stops are placed from the geometry, preload / stiffness short of
    /// straight, not read from the model.
    /// </summary>
    public static ChainStatics ForWing(PlanarModel scaleOne, LockingWingGeometry? geometry = null)
    {
        var w = geometry ?? new LockingWingGeometry();
        var g = w.ArmGeometry;
        var c = scaleOne.System.Cables;
        Stop?[] stops = w.HyperextensionStops
            ? [null, new Stop(-w.ElbowPreload / g.ElbowStopStiffness, g.ElbowStopStiffness), new Stop(-w.WristPreload / w.WristStopStiffness, w.WristStopStiffness)]
            : [null, null, null];
        return new ChainStatics(
            [new Segment(g.UpperLength, g.UpperMass), new Segment(g.LowerLength, g.LowerMass), new Segment(w.HandLength, w.HandMass)],
            [
                new Tendon(g.ShoulderAnchorBehind, g.ShoulderAnchorAbove, g.ShoulderInsertion, c[0].RestLength),
                new Tendon(g.ElbowPostBehind, g.ElbowPostAbove, g.ElbowInsertion, c[1].RestLength),
                new Tendon(w.WristPostBehind, w.WristPostAbove, w.WristInsertion, c[2].RestLength),
            ],
            g.AxialStiffness,
            9.81,
            stops);
    }

    /// <summary>
    /// Statics of a <see cref="ToggleArm"/>, with the rest lengths its model
    /// carries. The elbow's two stops are placed from the geometry, the
    /// straight stop <see cref="ToggleArm.StraightPress"/> / k short of
    /// straight, not read from the model.
    /// </summary>
    public static ChainStatics ForToggle(PlanarModel model, ToggleArmGeometry? geometry = null)
    {
        var g = geometry ?? new ToggleArmGeometry();
        var c = model.System.Cables;
        var straightStop = -ToggleArm.StraightPress(g) / g.ElbowStopStiffness;
        return new ChainStatics(
            [new Segment(g.UpperLength, g.UpperMass), new Segment(g.LowerLength, g.LowerMass)],
            [
                new Tendon(g.ShoulderAnchorBehind, g.ShoulderAnchorAbove, g.ShoulderInsertion, c[0].RestLength),
                new Tendon(g.ElbowPostBehind, g.ElbowPostAbove, g.ElbowInsertion, c[1].RestLength, g.ElbowInsertionAbove),
            ],
            g.AxialStiffness,
            9.81,
            [null, new Stop(straightStop, g.ElbowStopStiffness, -g.ElbowFoldStop)]);
    }

    /// <summary>The potential V at the pose, J, with tendon rest lengths scaled by <paramref name="scales"/>.</summary>
    public double Potential(IReadOnlyList<double> angles, IReadOnlyList<double> scales)
    {
        var n = links.Length;
        var u = new Vector2d[n];
        var joints = new Vector2d[n + 1];
        for (var i = 0; i < n; i++)
        {
            u[i] = new Vector2d(Math.Cos(angles[i]), Math.Sin(angles[i]));
            joints[i + 1] = joints[i] + (links[i].Length * u[i]);
        }

        var v = 0.0;
        for (var i = 0; i < n; i++)
        {
            v += gravity * links[i].Mass * (joints[i] + (0.5 * links[i].Length * u[i])).Y;
        }

        for (var j = 0; j < n; j++)
        {
            var t = tendons[j];
            var anchor = j == 0
                ? new Vector2d(-t.Behind, t.Above)
                : joints[j] - (t.Behind * u[j - 1]) + (t.Above * Perp(u[j - 1]));
            var length = (Insertion(joints, u, j) - anchor).Length;
            var rest = scales[j] * t.RestLength;
            if (length > rest)
            {
                v += 0.5 * (axialStiffness / rest) * (length - rest) * (length - rest);
            }
        }

        for (var j = 1; j < n; j++)
        {
            foreach (var past in new[] { StopPenetration(angles, j), FoldPenetration(angles, j) })
            {
                if (past > 0)
                {
                    v += 0.5 * stops[j]!.Value.Stiffness * past * past;
                }
            }
        }

        return v;
    }

    /// <summary>∂V/∂θₖ for every link angle, with tendon rest lengths scaled by <paramref name="scales"/>.</summary>
    public double[] Gradient(IReadOnlyList<double> angles, IReadOnlyList<double> scales)
    {
        var n = links.Length;
        var u = new Vector2d[n];
        var joints = new Vector2d[n + 1];
        for (var i = 0; i < n; i++)
        {
            u[i] = new Vector2d(Math.Cos(angles[i]), Math.Sin(angles[i]));
            joints[i + 1] = joints[i] + (links[i].Length * u[i]);
        }

        var d = new double[n];

        // ∂P/∂θₖ for a point P on link i: link k turning alone about Jₖ turns
        // its own points and carries the outer links along with Jₖ₊₁.
        Vector2d Moves(Vector2d point, int link, int k) =>
            k < link ? Perp(joints[k + 1] - joints[k]) : k == link ? Perp(point - joints[k]) : Vector2d.Zero;

        // Gravity: link i's center moves with every angle k ≤ i.
        for (var i = 0; i < n; i++)
        {
            var center = joints[i] + (0.5 * links[i].Length * u[i]);
            for (var k = 0; k <= i; k++)
            {
                d[k] += gravity * links[i].Mass * Moves(center, i, k).Y;
            }
        }

        // Tendon j: T·dir·(∂insertion − ∂anchor), the anchor on link j − 1 (ground for j = 0).
        for (var j = 0; j < n; j++)
        {
            var t = tendons[j];
            var anchor = j == 0
                ? new Vector2d(-t.Behind, t.Above)
                : joints[j] - (t.Behind * u[j - 1]) + (t.Above * Perp(u[j - 1]));
            var insertion = Insertion(joints, u, j);
            var span = insertion - anchor;
            var rest = scales[j] * t.RestLength;
            var length = span.Length;
            if (!(length > rest))
            {
                continue;
            }

            var tension = axialStiffness * (length - rest) / rest;
            var dir = span / length;
            for (var k = 0; k <= j; k++)
            {
                var moved = Moves(insertion, j, k) - (j > 0 ? Moves(anchor, j - 1, k) : Vector2d.Zero);
                d[k] += tension * Vector2d.Dot(dir, moved);
            }
        }

        // A hyperextension stop's torque turns link j down and link j − 1 up;
        // a fold stop's the other way.
        for (var j = 1; j < n; j++)
        {
            var past = StopPenetration(angles, j);
            if (past > 0)
            {
                var torque = stops[j]!.Value.Stiffness * past;
                d[j] += torque;
                d[j - 1] -= torque;
            }

            var folded = FoldPenetration(angles, j);
            if (folded > 0)
            {
                var torque = stops[j]!.Value.Stiffness * folded;
                d[j] -= torque;
                d[j - 1] += torque;
            }
        }

        return d;
    }

    /// <summary>Hessian of V, symmetrized, by central differences of the exact gradient.</summary>
    public double[,] Hessian(IReadOnlyList<double> angles, IReadOnlyList<double> scales)
    {
        const double h = 1e-6;
        var n = links.Length;
        var hessian = new double[n, n];
        var probe = angles.ToArray();
        for (var k = 0; k < n; k++)
        {
            probe[k] = angles[k] + h;
            var plus = Gradient(probe, scales);
            probe[k] = angles[k] - h;
            var minus = Gradient(probe, scales);
            probe[k] = angles[k];
            for (var i = 0; i < n; i++)
            {
                hessian[i, k] = (plus[i] - minus[i]) / (2 * h);
            }
        }

        for (var i = 0; i < n; i++)
        {
            for (var k = 0; k < i; k++)
            {
                var mean = 0.5 * (hessian[i, k] + hessian[k, i]);
                hessian[i, k] = mean;
                hessian[k, i] = mean;
            }
        }

        return hessian;
    }

    /// <summary>True if the Hessian is positive definite (a Cholesky factor exists): a stable rest pose if ∇V = 0 there.</summary>
    public bool IsStable(IReadOnlyList<double> angles, IReadOnlyList<double> scales)
    {
        var a = Hessian(angles, scales);
        var n = links.Length;
        for (var j = 0; j < n; j++)
        {
            var diagonal = a[j, j];
            for (var k = 0; k < j; k++)
            {
                diagonal -= a[j, k] * a[j, k];
            }

            if (!(diagonal > 0))
            {
                return false;
            }

            a[j, j] = Math.Sqrt(diagonal);
            for (var i = j + 1; i < n; i++)
            {
                var sum = a[i, j];
                for (var k = 0; k < j; k++)
                {
                    sum -= a[i, k] * a[j, k];
                }

                a[i, j] = sum / a[j, j];
            }
        }

        return true;
    }

    /// <summary>
    /// The stable rest pose for the given scales, followed from straight out
    /// at scale 1 in small scale steps so each Newton solve starts next to
    /// its answer.
    /// </summary>
    public double[] RestPose(IReadOnlyList<double> scales)
    {
        const int steps = 100;
        var n = links.Length;
        var angles = new double[n];
        var step = new double[n];
        for (var s = 1; s <= steps; s++)
        {
            var f = (double)s / steps;
            for (var j = 0; j < n; j++)
            {
                step[j] = 1 + ((scales[j] - 1) * f);
            }

            angles = Newton(step, angles);
        }

        if (!IsStable(angles, scales))
        {
            throw new InvalidOperationException("The followed rest pose is not stable: the scales are past a fold.");
        }

        return angles;
    }

    /// <summary>
    /// Tendon <paramref name="tendon"/>'s lock threshold, the others at
    /// scale 1. Follows the branch of rest poses from straight out, stepping
    /// that tendon's link angle down and solving for the other angles and
    /// the scale; the scale peaks at the fold, refined by golden section.
    /// </summary>
    public ChainFold Fold(int tendon)
    {
        var n = links.Length;
        if ((uint)tendon >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(tendon), tendon, $"The chain has {n} tendons.");
        }

        // Unknowns x: the other angles in order, then the scale.
        double[] Angles(double driven, double[] x)
        {
            var angles = new double[n];
            for (int i = 0, k = 0; i < n; i++)
            {
                angles[i] = i == tendon ? driven : x[k++];
            }

            return angles;
        }

        double[] Residual(double driven, double[] x)
        {
            var scales = Enumerable.Repeat(1.0, n).ToArray();
            scales[tendon] = x[^1];
            return Gradient(Angles(driven, x), scales);
        }

        double[] Solve(double driven, double[] guess)
        {
            var x = (double[])guess.Clone();
            for (var iteration = 0; iteration < 100; iteration++)
            {
                var r = Residual(driven, x);
                var jacobian = new double[n, n];
                const double h = 1e-7;
                for (var c = 0; c < n; c++)
                {
                    var probe = (double[])x.Clone();
                    probe[c] += h;
                    var rp = Residual(driven, probe);
                    for (var i = 0; i < n; i++)
                    {
                        jacobian[i, c] = (rp[i] - r[i]) / h;
                    }
                }

                var dx = SolveLinear(jacobian, r);
                var size = 0.0;
                for (var i = 0; i < n; i++)
                {
                    x[i] -= dx[i];
                    size += Math.Abs(dx[i]);
                }

                if (size < 1e-14)
                {
                    return x;
                }
            }

            throw new InvalidOperationException("Branch point did not converge.");
        }

        // Walk the driven angle down in 0.5° steps until the scale starts to fall.
        var start = new double[n];
        start[^1] = 1;
        var points = new List<(double Driven, double[] X)> { (0, start) };
        var step = -0.5 * Math.PI / 180;
        while (true)
        {
            var last = points[^1];
            var driven = last.Driven + step;
            points.Add((driven, Solve(driven, last.X)));
            if (points.Count >= 3 && points[^1].X[^1] < points[^2].X[^1])
            {
                break;
            }

            if (driven < -Math.PI / 2)
            {
                throw new InvalidOperationException("No fold before the link hangs straight down.");
            }
        }

        var lo = points[^1].Driven;
        var hi = points[^3].Driven;
        var seed = points[^2].X;
        double ScaleAt(double driven) => Solve(driven, seed)[^1];

        var phi = (Math.Sqrt(5) - 1) / 2;
        var x1 = hi - (phi * (hi - lo));
        var x2 = lo + (phi * (hi - lo));
        var f1 = ScaleAt(x1);
        var f2 = ScaleAt(x2);
        while (hi - lo > 1e-9)
        {
            if (f1 > f2)
            {
                lo = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - (phi * (hi - lo));
                f1 = ScaleAt(x1);
            }
            else
            {
                hi = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + (phi * (hi - lo));
                f2 = ScaleAt(x2);
            }
        }

        var peak = 0.5 * (lo + hi);
        var solved = Solve(peak, seed);
        return new ChainFold(tendon, solved[^1], Angles(peak, solved));
    }

    /// <summary>
    /// The torque joint <paramref name="joint"/>'s hyperextension stop
    /// carries at the pose, N·m (pushing the outer link down); zero off its
    /// stop or without one.
    /// </summary>
    public double StopTorque(IReadOnlyList<double> angles, int joint) =>
        joint > 0 && StopPenetration(angles, joint) is var past && past > 0 ? stops[joint]!.Value.Stiffness * past : 0;

    /// <summary>
    /// The torque joint <paramref name="joint"/>'s fold stop carries at the
    /// pose, N·m (pushing the outer link up); zero off its stop or without one.
    /// </summary>
    public double FoldStopTorque(IReadOnlyList<double> angles, int joint) =>
        joint > 0 && FoldPenetration(angles, joint) is var past && past > 0 ? stops[joint]!.Value.Stiffness * past : 0;

    /// <summary>
    /// The stable rest pose nearest <paramref name="guess"/> (world link
    /// angles) at the given scales, by Newton's method from the guess.
    /// </summary>
    public double[] RestPoseNear(IReadOnlyList<double> guess, IReadOnlyList<double> scales)
    {
        var angles = Newton(scales, guess.ToArray());
        if (!IsStable(angles, scales))
        {
            throw new InvalidOperationException("The rest pose found from this guess is not stable.");
        }

        return angles;
    }

    /// <summary>
    /// The energy barrier between two rest poses that differ at joint
    /// <paramref name="joint"/>, at scale 1. Holding that joint's angle φ
    /// and letting every other joint settle gives the profile V*(φ); its
    /// highest point between the poses is the lowest pass from one to the
    /// other, the saddle of V. The profile is followed from
    /// <paramref name="from"/> to <paramref name="to"/> in 0.25° steps and
    /// its peak refined by golden section.
    /// </summary>
    public ChainBarrier Barrier(int joint, IReadOnlyList<double> from, IReadOnlyList<double> to)
    {
        var n = links.Length;
        if (joint < 1 || joint >= n)
        {
            throw new ArgumentOutOfRangeException(nameof(joint), joint, $"Joints run from 1 to {n - 1}.");
        }

        var scales = Enumerable.Repeat(1.0, n).ToArray();

        // Joint coordinates q: q₀ = θ₀, qᵢ = θᵢ − θᵢ₋₁. ∂V/∂qᵢ = Σₖ≥ᵢ ∂V/∂θₖ.
        double[] ToJoints(IReadOnlyList<double> angles)
        {
            var q = new double[n];
            for (var i = 0; i < n; i++)
            {
                q[i] = angles[i] - (i > 0 ? angles[i - 1] : 0);
            }

            return q;
        }

        double[] ToAngles(double[] q)
        {
            var angles = new double[n];
            for (var i = 0; i < n; i++)
            {
                angles[i] = q[i] + (i > 0 ? angles[i - 1] : 0);
            }

            return angles;
        }

        double[] Residual(double[] q)
        {
            var g = Gradient(ToAngles(q), scales);
            var r = new double[n];
            for (var i = n - 1; i >= 0; i--)
            {
                r[i] = g[i] + (i < n - 1 ? r[i + 1] : 0);
            }

            return r;
        }

        // Settles every joint but the held one, from a guess with the held angle set.
        double[] Settle(double[] guess)
        {
            var free = Enumerable.Range(0, n).Where(i => i != joint).ToArray();
            var q = (double[])guess.Clone();
            for (var iteration = 0; iteration < 100; iteration++)
            {
                var r = Residual(q);
                var jacobian = new double[free.Length, free.Length];
                const double h = 1e-7;
                for (var c = 0; c < free.Length; c++)
                {
                    var probe = (double[])q.Clone();
                    probe[free[c]] += h;
                    var rp = Residual(probe);
                    for (var i = 0; i < free.Length; i++)
                    {
                        jacobian[i, c] = (rp[free[i]] - r[free[i]]) / h;
                    }
                }

                var dx = SolveLinear(jacobian, free.Select(i => r[i]).ToArray());
                var size = 0.0;
                for (var c = 0; c < free.Length; c++)
                {
                    q[free[c]] -= dx[c];
                    size += Math.Abs(dx[c]);
                }

                if (size < 1e-14)
                {
                    return q;
                }
            }

            throw new InvalidOperationException("Profile point did not converge.");
        }

        double[] At(double angle, double[] guess)
        {
            var start = (double[])guess.Clone();
            start[joint] = angle;
            return Settle(start);
        }

        var a = ToJoints(from);
        var b = ToJoints(to);
        var steps = (int)Math.Ceiling(Math.Abs(b[joint] - a[joint]) / (0.25 * Math.PI / 180));
        var best = (Index: 0, Potential: double.NegativeInfinity, Q: a);
        var profile = new List<double[]>();
        var q = a;
        for (var s = 0; s <= steps; s++)
        {
            q = At(a[joint] + ((b[joint] - a[joint]) * s / steps), q);
            profile.Add(q);
            var v = Potential(ToAngles(q), scales);
            if (v > best.Potential)
            {
                best = (s, v, q);
            }
        }

        if (best.Index == 0 || best.Index == steps)
        {
            throw new InvalidOperationException("The profile peaks at an end: the two poses are not separated by a barrier.");
        }

        var seed = profile[best.Index];
        double PotentialAt(double angle) => Potential(ToAngles(At(angle, seed)), scales);
        var lo = Math.Min(profile[best.Index - 1][joint], profile[best.Index + 1][joint]);
        var hi = Math.Max(profile[best.Index - 1][joint], profile[best.Index + 1][joint]);
        var golden = (Math.Sqrt(5) - 1) / 2;
        var x1 = hi - (golden * (hi - lo));
        var x2 = lo + (golden * (hi - lo));
        var f1 = PotentialAt(x1);
        var f2 = PotentialAt(x2);
        while (hi - lo > 1e-10)
        {
            if (f1 > f2)
            {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - (golden * (hi - lo));
                f1 = PotentialAt(x1);
            }
            else
            {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + (golden * (hi - lo));
                f2 = PotentialAt(x2);
            }
        }

        var peak = At(0.5 * (lo + hi), seed);
        var peakAngles = ToAngles(peak);
        return new ChainBarrier(joint, peak[joint], Potential(peakAngles, scales), peakAngles);
    }

    /// <summary>How far joint j is past its hyperextension stop, rad; −∞ without one.</summary>
    private double StopPenetration(IReadOnlyList<double> angles, int joint) =>
        stops[joint] is { } stop ? angles[joint] - angles[joint - 1] - stop.Angle : double.NegativeInfinity;

    /// <summary>How far joint j is folded past its fold stop, rad; −∞ without one.</summary>
    private double FoldPenetration(IReadOnlyList<double> angles, int joint) =>
        stops[joint] is { } stop ? stop.Fold - (angles[joint] - angles[joint - 1]) : double.NegativeInfinity;

    /// <summary>Tendon j's insertion on link j, along its axis and above it.</summary>
    private Vector2d Insertion(Vector2d[] joints, Vector2d[] u, int j) =>
        joints[j] + (tendons[j].Insertion * u[j]) + (tendons[j].InsertionAbove * Perp(u[j]));

    private double[] Newton(IReadOnlyList<double> scales, double[] guess)
    {
        var angles = (double[])guess.Clone();
        for (var i = 0; i < 100; i++)
        {
            var step = SolveLinear(Hessian(angles, scales), Gradient(angles, scales));
            var size = 0.0;
            for (var k = 0; k < angles.Length; k++)
            {
                angles[k] -= step[k];
                size += Math.Abs(step[k]);
            }

            if (size < 1e-13)
            {
                return angles;
            }
        }

        throw new InvalidOperationException("Rest pose did not converge.");
    }

    /// <summary>Solves A·x = b by Gaussian elimination with partial pivoting; A and b are left unchanged.</summary>
    private static double[] SolveLinear(double[,] matrix, double[] rhs)
    {
        var n = rhs.Length;
        var a = (double[,])matrix.Clone();
        var b = (double[])rhs.Clone();
        for (var col = 0; col < n; col++)
        {
            var pivot = col;
            for (var row = col + 1; row < n; row++)
            {
                if (Math.Abs(a[row, col]) > Math.Abs(a[pivot, col]))
                {
                    pivot = row;
                }
            }

            if (pivot != col)
            {
                for (var k = 0; k < n; k++)
                {
                    (a[col, k], a[pivot, k]) = (a[pivot, k], a[col, k]);
                }

                (b[col], b[pivot]) = (b[pivot], b[col]);
            }

            for (var row = col + 1; row < n; row++)
            {
                var factor = a[row, col] / a[col, col];
                for (var k = col; k < n; k++)
                {
                    a[row, k] -= factor * a[col, k];
                }

                b[row] -= factor * b[col];
            }
        }

        var x = new double[n];
        for (var row = n - 1; row >= 0; row--)
        {
            var sum = b[row];
            for (var k = row + 1; k < n; k++)
            {
                sum -= a[row, k] * x[k];
            }

            x[row] = sum / a[row, row];
        }

        return x;
    }

    private static Vector2d Perp(Vector2d v) => new(-v.Y, v.X);

    private readonly record struct Segment(double Length, double Mass);

    /// <summary>
    /// Stops on a joint angle: where the hyperextension stop starts to push,
    /// rad, their torsional stiffness, N·m/rad, and where the fold stop
    /// starts to push, rad (−∞ for none).
    /// </summary>
    private readonly record struct Stop(double Angle, double Stiffness, double Fold = double.NegativeInfinity);

    /// <summary>A tendon: its anchor behind and above its joint, its insertion along the next link (and above its axis), and its rest length at scale 1.</summary>
    private readonly record struct Tendon(double Behind, double Above, double Insertion, double RestLength, double InsertionAbove = 0);
}
