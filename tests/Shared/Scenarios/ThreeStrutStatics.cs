using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>Where the starting shape gives way: the muscle's shortening, the leans just before, and its tension then.</summary>
public sealed record ThreeStrutSnap(double Shortening, double[] Leans, double MuscleTension);

/// <summary>
/// The statics prediction for M9 (PLAN.md, stage 2): the <see cref="ThreeStrut"/>
/// frame in its three lean angles αᵢ (positive leans right), written
/// independently of the simulator. Each post is pinned at its foot, so its
/// pose is its lean alone, and the potential is
/// <code>
/// V(α; Δ) = Σ m·g·(l/2)·cos αᵢ + Σ ½·k·(L − L0)²   (stretched cables only)
/// </code>
/// where a muscle's L0 is its tendon's rest length plus its contractile
/// length ℓ0 − Δ, after shortening by Δ, and k = EA / L0 is the tendon's
/// own. Rest shapes are zeros of ∇V, stable where the Hessian is positive
/// definite. The gradient and Hessian are exact.
///
/// Shortening the muscle moves the starting shape along a branch of rest
/// shapes until the branch ends; <see cref="FindSnap"/> follows it there.
/// The frame then settles in the snapped shape, found by
/// <see cref="Settle"/> from the starting shape's mirror image. Settling
/// caps each step, so it walks downhill rather than jumping: the posts are
/// pinned with no ground contact, and an unlimited Newton step can swing
/// them past the guy to hang below the ground, the global minimum.
/// </summary>
public sealed class ThreeStrutStatics
{
    private readonly ThreeStrutGeometry g;
    private readonly double gravity;
    private readonly End[] ends;
    private readonly string[] ids;
    private readonly double[] rest;
    private readonly double[] stiffness;
    private readonly bool[] isMuscle;
    private readonly int muscle;

    /// <param name="geometry">The frame's dimensions.</param>
    /// <param name="cables">Its cables, read for their ends, rest lengths, stiffness and muscle.</param>
    /// <param name="gravity">Magnitude of gravity, straight down, m/s².</param>
    public ThreeStrutStatics(ThreeStrutGeometry geometry, IEnumerable<Cable> cables, double gravity = 9.81)
    {
        g = geometry ?? throw new ArgumentNullException(nameof(geometry));
        this.gravity = gravity;
        var list = cables.ToArray();
        ids = list.Select(c => c.Id).ToArray();
        rest = list.Select(c => c.RestLength).ToArray();
        stiffness = list.Select(c => c.Stiffness).ToArray();
        isMuscle = list.Select(c => c.Muscle is not null).ToArray();
        ends = list.SelectMany(c => new[] { ToEnd(c.A), ToEnd(c.B) }).ToArray();
        muscle = Array.FindIndex(isMuscle, m => m);
        if (muscle < 0 || isMuscle.Count(m => m) != 1)
        {
            throw new ArgumentException("The frame needs exactly one muscle.", nameof(cables));
        }

        InitialContractileLength = list[muscle].Muscle!.InitialLength;
    }

    /// <summary>The muscle's contractile length before it shortens, m.</summary>
    public double InitialContractileLength { get; }

    /// <summary>The starting shape: every post leaning left by the geometry's lean.</summary>
    public double[] Start => [-g.Lean, -g.Lean, -g.Lean];

    /// <summary>Potential energy V(α; Δ), J.</summary>
    public double Energy(ReadOnlySpan<double> leans, double shortening)
    {
        var total = 0.0;
        for (var i = 0; i < 3; i++)
        {
            total += g.PostMass * gravity * (g.PostLength / 2) * Math.Cos(leans[i]);
        }

        for (var c = 0; c < ids.Length; c++)
        {
            var stretch = Length(c, leans) - RestLength(c, shortening);
            if (stretch > 0)
            {
                total += 0.5 * stiffness[c] * stretch * stretch;
            }
        }

        return total;
    }

    /// <summary>∇V, J/rad.</summary>
    public double[] Gradient(ReadOnlySpan<double> leans, double shortening)
    {
        var gradient = new double[3];
        for (var i = 0; i < 3; i++)
        {
            gradient[i] = -g.PostMass * gravity * (g.PostLength / 2) * Math.Sin(leans[i]);
        }

        for (var c = 0; c < ids.Length; c++)
        {
            var (length, first, _) = LengthDerivatives(c, leans);
            var stretch = length - RestLength(c, shortening);
            if (stretch > 0)
            {
                for (var i = 0; i < 3; i++)
                {
                    gradient[i] += stiffness[c] * stretch * first[i];
                }
            }
        }

        return gradient;
    }

    /// <summary>
    /// The Hessian of V, row-major 3 × 3, J/rad². With <paramref name="without"/>,
    /// that cable is left out: how stiff the shape would be without it.
    /// </summary>
    public double[] Hessian(ReadOnlySpan<double> leans, double shortening, string? without = null)
    {
        var hessian = new double[9];
        for (var i = 0; i < 3; i++)
        {
            hessian[(3 * i) + i] = -g.PostMass * gravity * (g.PostLength / 2) * Math.Cos(leans[i]);
        }

        for (var c = 0; c < ids.Length; c++)
        {
            if (ids[c] == without)
            {
                continue;
            }

            var (length, first, second) = LengthDerivatives(c, leans);
            var stretch = length - RestLength(c, shortening);
            if (stretch > 0)
            {
                for (var i = 0; i < 3; i++)
                {
                    for (var j = 0; j < 3; j++)
                    {
                        hessian[(3 * i) + j] += stiffness[c] * ((first[i] * first[j]) + (stretch * second[(3 * i) + j]));
                    }
                }
            }
        }

        return hessian;
    }

    /// <summary>The Hessian's smallest eigenvalue, J/rad²: positive where the shape is stable.</summary>
    public double SmallestEigenvalue(ReadOnlySpan<double> leans, double shortening, string? without = null)
    {
        var values = new double[3];
        SymmetricEigen.Decompose(Hessian(leans, shortening, without), 3, values, new double[9]);
        return values.Min();
    }

    /// <summary>Tension in cable <paramref name="id"/> at rest in this shape, N.</summary>
    public double Tension(string id, ReadOnlySpan<double> leans, double shortening)
    {
        var c = Array.IndexOf(ids, id);
        if (c < 0)
        {
            throw new ArgumentException($"No cable '{id}'.", nameof(id));
        }

        return Math.Max(0, stiffness[c] * (Length(c, leans) - RestLength(c, shortening)));
    }

    /// <summary>The muscle's tension at rest in this shape, N.</summary>
    public double MuscleTension(ReadOnlySpan<double> leans, double shortening) => Tension(ids[muscle], leans, shortening);

    /// <summary>
    /// The rest shape that the frame settles into from <paramref name="start"/>:
    /// damped Newton steps on V, each at most <paramref name="maxStep"/> rad,
    /// until the gradient vanishes.
    /// </summary>
    public double[] Settle(ReadOnlySpan<double> start, double shortening, double maxStep = 0.02)
    {
        var x = start.ToArray();
        for (var iteration = 0; iteration < 2000; iteration++)
        {
            var gradient = Gradient(x, shortening);
            if (Norm(gradient) < 1e-13)
            {
                break;
            }

            var step = DampedNewtonStep(Hessian(x, shortening), gradient);
            var scale = Math.Min(1.0, maxStep / step.Max(Math.Abs));
            var energy = Energy(x, shortening);
            var t = 1.0;
            while (t > 1e-12)
            {
                var y = x.Select((v, i) => v - (t * scale * step[i])).ToArray();
                if (Energy(y, shortening) <= energy + 1e-15)
                {
                    x = y;
                    break;
                }

                t /= 2;
            }

            if (t <= 1e-12)
            {
                break;
            }
        }

        return x;
    }

    /// <summary>
    /// Follows the starting shape as the muscle shortens, in steps of
    /// <paramref name="step"/>, until settling from the last shape lands
    /// leaning right; then bisects for the last shortening where it still
    /// leans left, to <paramref name="tolerance"/>.
    /// </summary>
    public ThreeStrutSnap FindSnap(double step = 1e-3, double tolerance = 1e-10)
    {
        var last = Settle(Start, 0);
        double below = 0, above = double.NaN;
        for (var d = step; d < InitialContractileLength; d += step)
        {
            var next = Settle(last, d);
            if (next.Average() > 0)
            {
                above = d;
                break;
            }

            last = next;
            below = d;
        }

        if (double.IsNaN(above))
        {
            throw new InvalidOperationException("The frame never snapped.");
        }

        while (above - below > tolerance)
        {
            var middle = (below + above) / 2;
            var next = Settle(last, middle);
            if (next.Average() > 0)
            {
                above = middle;
            }
            else
            {
                below = middle;
                last = next;
            }
        }

        return new ThreeStrutSnap(below, last, MuscleTension(last, below));
    }

    /// <summary>The snapped shape after shortening by <paramref name="shortening"/>, settled from the starting shape's mirror image.</summary>
    public double[] Snapped(double shortening) => Settle([g.Lean, g.Lean, g.Lean], shortening);

    /// <summary>
    /// How far the muscle must shorten, past <paramref name="snap"/>, for the
    /// snapped shape to leave it at <paramref name="tension"/>. More
    /// shortening stretches its tendon more, so the tension rises with it.
    /// </summary>
    public double StopFor(double tension, ThreeStrutSnap snap, double tolerance = 1e-12)
    {
        double low = snap.Shortening, high = InitialContractileLength;
        if (MuscleTension(Snapped(low), low) > tension)
        {
            throw new InvalidOperationException("The snapped shape already leaves the muscle tighter than that.");
        }

        while (high - low > tolerance)
        {
            var middle = (low + high) / 2;
            if (MuscleTension(Snapped(middle), middle) > tension)
            {
                high = middle;
            }
            else
            {
                low = middle;
            }
        }

        return low;
    }

    private double RestLength(int c, double shortening) =>
        isMuscle[c] ? rest[c] + InitialContractileLength - shortening : rest[c];

    private double Length(int c, ReadOnlySpan<double> leans) =>
        (Point(ends[2 * c], leans) - Point(ends[(2 * c) + 1], leans)).Length;

    /// <summary>L, ∂L/∂αᵢ and ∂²L/∂αᵢ∂αⱼ (row-major) for cable <paramref name="c"/>.</summary>
    private (double Length, double[] First, double[] Second) LengthDerivatives(int c, ReadOnlySpan<double> leans)
    {
        var p = ends[2 * c];
        var q = ends[(2 * c) + 1];
        var d = Point(p, leans) - Point(q, leans);
        var length = d.Length;

        // d = p − q: each end moves only with its own post's lean.
        var dd = new Vector2d[3];
        var dd2 = new Vector2d[3];
        foreach (var (end, sign) in new[] { (p, 1.0), (q, -1.0) })
        {
            if (end.Post >= 0)
            {
                var (first, second) = PointDerivatives(end, leans[end.Post]);
                dd[end.Post] += sign * first;
                dd2[end.Post] += sign * second;
            }
        }

        var dL = new double[3];
        for (var i = 0; i < 3; i++)
        {
            dL[i] = Vector2d.Dot(d, dd[i]) / length;
        }

        var d2L = new double[9];
        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                var curvature = i == j ? Vector2d.Dot(d, dd2[i]) : 0.0;
                d2L[(3 * i) + j] = ((Vector2d.Dot(dd[i], dd[j]) + curvature) / length) - (dL[i] * dL[j] / length);
            }
        }

        return (length, dL, d2L);
    }

    /// <summary>
    /// A point fixed to post i, at offset (u, v) from its foot along and
    /// across it: the post's axis is (sin α, cos α), so the point is at
    /// foot + (u·sin α − v·cos α, u·cos α + v·sin α).
    /// </summary>
    private Vector2d Point(End end, ReadOnlySpan<double> leans)
    {
        if (end.Post < 0)
        {
            return end.Offset;
        }

        var a = leans[end.Post];
        var (u, v) = (end.Offset.X, end.Offset.Y);
        return ThreeStrut.Foot(g, end.Post) + new Vector2d((u * Math.Sin(a)) - (v * Math.Cos(a)), (u * Math.Cos(a)) + (v * Math.Sin(a)));
    }

    private static (Vector2d First, Vector2d Second) PointDerivatives(End end, double a)
    {
        var (u, v) = (end.Offset.X, end.Offset.Y);
        return (
            new Vector2d((u * Math.Cos(a)) + (v * Math.Sin(a)), (-u * Math.Sin(a)) + (v * Math.Cos(a))),
            new Vector2d((-u * Math.Sin(a)) + (v * Math.Cos(a)), (-u * Math.Cos(a)) - (v * Math.Sin(a))));
    }

    /// <summary>A cable end: on a post (offset from its foot, along and across) or fixed on the ground (Post −1).</summary>
    private readonly record struct End(int Post, Vector2d Offset);

    private End ToEnd(Attachment attachment)
    {
        if (attachment.IsGround)
        {
            return new End(-1, attachment.Point);
        }

        var post = Array.IndexOf(ThreeStrut.Posts, attachment.LinkId);
        if (post < 0)
        {
            throw new ArgumentException($"Cable end on unknown link '{attachment.LinkId}'.");
        }

        // A link's body x runs along it from end A (the foot) at −l/2.
        return new End(post, new Vector2d(attachment.Point.X + (g.PostLength / 2), attachment.Point.Y));
    }

    /// <summary>Solves (H + λI)·x = ∇V with the smallest λ ≥ 0 that makes H + λI positive definite.</summary>
    private static double[] DampedNewtonStep(double[] hessian, double[] gradient)
    {
        var lambda = 0.0;
        for (var attempt = 0; attempt < 80; attempt++)
        {
            var shifted = (double[])hessian.Clone();
            for (var i = 0; i < 3; i++)
            {
                shifted[(3 * i) + i] += lambda;
            }

            if (CholeskySolve(shifted, gradient) is { } step)
            {
                return step;
            }

            lambda = lambda == 0 ? 1e-6 : lambda * 10;
        }

        throw new InvalidOperationException("Could not make the Hessian positive definite.");
    }

    private static double[]? CholeskySolve(double[] a, double[] b)
    {
        const int n = 3;
        var l = new double[9];
        for (var i = 0; i < n; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                var s = a[(n * i) + j];
                for (var k = 0; k < j; k++)
                {
                    s -= l[(n * i) + k] * l[(n * j) + k];
                }

                if (i == j)
                {
                    if (!(s > 0))
                    {
                        return null;
                    }

                    l[(n * i) + i] = Math.Sqrt(s);
                }
                else
                {
                    l[(n * i) + j] = s / l[(n * j) + j];
                }
            }
        }

        var y = new double[n];
        for (var i = 0; i < n; i++)
        {
            var s = b[i];
            for (var k = 0; k < i; k++)
            {
                s -= l[(n * i) + k] * y[k];
            }

            y[i] = s / l[(n * i) + i];
        }

        var x = new double[n];
        for (var i = n - 1; i >= 0; i--)
        {
            var s = y[i];
            for (var k = i + 1; k < n; k++)
            {
                s -= l[(n * k) + i] * x[k];
            }

            x[i] = s / l[(n * i) + i];
        }

        return x;
    }

    private static double Norm(double[] v) => Math.Sqrt(v.Sum(x => x * x));
}
