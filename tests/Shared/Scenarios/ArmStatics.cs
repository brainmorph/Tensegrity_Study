using Tensegrity.Core.Mathematics;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>A lock threshold: the rest-length scale where the stable pose vanishes, and the pose there.</summary>
public readonly record struct ArmFold(double Scale, double UpperAngle, double LowerAngle);

/// <summary>
/// The geometric prediction for test 10 (PLAN.md section 7): statics of the
/// <see cref="LockingArm"/> in its two joint angles, written independently
/// of the simulator. The simulator works in maximal coordinates with
/// Lagrange-multiplier pins; this works with the potential
/// <code>
/// V(θ1, θ2) = g·(m1·y1 + m2·y2) + Σ ½·k·(L − s·L0)²   (stretched tendons only)
/// </code>
/// where θ1 and θ2 are the world angles of the upper and lower links,
/// s scales a tendon's rest length L0, and k = EA / (s·L0). Rest poses are
/// zeros of ∇V; a rest pose is stable (locked) where the Hessian is
/// positive definite.
///
/// Lengthening a tendon moves the stable pose down a branch of rest poses.
/// Along the branch the scale s rises to a maximum and falls again on the
/// unstable side: that maximum is the fold, the threshold past which no
/// stable pose is left and the arm collapses. Gradients are exact; the
/// Hessian is a central difference of them.
/// </summary>
public sealed class ArmStatics
{
    private readonly LockingArmGeometry g;
    private readonly double gravity;
    private readonly double shoulderRest;
    private readonly double elbowRest;

    /// <param name="geometry">The arm's dimensions.</param>
    /// <param name="shoulderRest">Shoulder tendon rest length at scale 1, m.</param>
    /// <param name="elbowRest">Elbow tendon rest length at scale 1, m.</param>
    /// <param name="gravity">Magnitude of gravity, straight down, m/s².</param>
    public ArmStatics(LockingArmGeometry geometry, double shoulderRest, double elbowRest, double gravity = 9.81)
    {
        g = geometry;
        this.shoulderRest = shoulderRest;
        this.elbowRest = elbowRest;
        this.gravity = gravity;
    }

    /// <summary>Statics for the rest lengths a generated arm carries at scale 1.</summary>
    public static ArmStatics For(PlanarModel scaleOne, LockingArmGeometry? geometry = null) =>
        new(geometry ?? new LockingArmGeometry(), scaleOne.System.Cables[0].RestLength, scaleOne.System.Cables[1].RestLength);

    /// <summary>∂V/∂θ1 and ∂V/∂θ2 at the pose, with rest lengths scaled by <paramref name="shoulderScale"/> and <paramref name="elbowScale"/>.</summary>
    public (double D1, double D2) Gradient(double t1, double t2, double shoulderScale, double elbowScale)
    {
        var u1 = new Vector2d(Math.Cos(t1), Math.Sin(t1));
        var n1 = Perp(u1);
        var u2 = new Vector2d(Math.Cos(t2), Math.Sin(t2));
        var anchor = new Vector2d(-g.ShoulderAnchorBehind, g.ShoulderAnchorAbove);
        var insertS = g.ShoulderInsertion * u1;
        var elbow = g.UpperLength * u1;
        var post = elbow - (g.ElbowPostBehind * u1) + (g.ElbowPostAbove * n1);
        var insertE = elbow + (g.ElbowInsertion * u2);
        var c1 = 0.5 * g.UpperLength * u1;
        var c2 = elbow + (0.5 * g.LowerLength * u2);

        // Gravity: V_g = g·(m1·y1 + m2·y2), and a point rotating about q moves as perp(p − q).
        var d1 = gravity * ((g.UpperMass * Perp(c1).Y) + (g.LowerMass * Perp(elbow).Y));
        var d2 = gravity * g.LowerMass * Perp(c2 - elbow).Y;

        // Shoulder tendon: anchor fixed, insertion turns with θ1 about the shoulder.
        var spanS = insertS - anchor;
        var tS = Tension(spanS.Length, shoulderScale * shoulderRest);
        d1 += tS * Vector2d.Dot(spanS / spanS.Length, Perp(insertS));

        // Elbow tendon: the post turns with θ1; the insertion turns with θ1 about the shoulder and θ2 about the elbow.
        var spanE = insertE - post;
        var tE = Tension(spanE.Length, elbowScale * elbowRest);
        var dirE = spanE / spanE.Length;
        d1 += tE * Vector2d.Dot(dirE, Perp(elbow) - Perp(post));
        d2 += tE * Vector2d.Dot(dirE, Perp(insertE - elbow));
        return (d1, d2);
    }

    /// <summary>Hessian of V at the pose, by central differences of the exact gradient.</summary>
    public (double H11, double H12, double H22) Hessian(double t1, double t2, double shoulderScale, double elbowScale)
    {
        const double h = 1e-6;
        var a = Gradient(t1 + h, t2, shoulderScale, elbowScale);
        var b = Gradient(t1 - h, t2, shoulderScale, elbowScale);
        var c = Gradient(t1, t2 + h, shoulderScale, elbowScale);
        var d = Gradient(t1, t2 - h, shoulderScale, elbowScale);
        var h12 = 0.5 * (((a.D2 - b.D2) / (2 * h)) + ((c.D1 - d.D1) / (2 * h)));
        return ((a.D1 - b.D1) / (2 * h), h12, (c.D2 - d.D2) / (2 * h));
    }

    /// <summary>True if the Hessian is positive definite: the pose is a stable rest pose if ∇V = 0 there.</summary>
    public bool IsStable(double t1, double t2, double shoulderScale, double elbowScale)
    {
        var (h11, h12, h22) = Hessian(t1, t2, shoulderScale, elbowScale);
        return h11 > 0 && (h11 * h22) - (h12 * h12) > 0;
    }

    /// <summary>
    /// The stable rest pose (θ1, θ2) for the given scales, followed from
    /// straight out at scale 1 in small scale steps so that each Newton
    /// solve starts next to its answer. A lengthened tendon can be slack
    /// straight out, where Newton alone would diverge.
    /// </summary>
    public (double UpperAngle, double LowerAngle) RestPose(double shoulderScale, double elbowScale)
    {
        const int steps = 100;
        double t1 = 0, t2 = 0;
        for (var i = 1; i <= steps; i++)
        {
            var f = (double)i / steps;
            (t1, t2) = Newton(1 + ((shoulderScale - 1) * f), 1 + ((elbowScale - 1) * f), t1, t2);
        }

        if (!IsStable(t1, t2, shoulderScale, elbowScale))
        {
            throw new InvalidOperationException("The followed rest pose is not stable: the scales are past the fold.");
        }

        return (t1, t2);
    }

    /// <summary>Rest pose near the guess, by Newton's method on ∇V = 0.</summary>
    private (double, double) Newton(double shoulderScale, double elbowScale, double t1, double t2)
    {
        for (var i = 0; i < 100; i++)
        {
            var (d1, d2) = Gradient(t1, t2, shoulderScale, elbowScale);
            var (h11, h12, h22) = Hessian(t1, t2, shoulderScale, elbowScale);
            var det = (h11 * h22) - (h12 * h12);
            var s1 = ((h22 * d1) - (h12 * d2)) / det;
            var s2 = ((h11 * d2) - (h12 * d1)) / det;
            t1 -= s1;
            t2 -= s2;
            if (Math.Abs(s1) + Math.Abs(s2) < 1e-13)
            {
                return (t1, t2);
            }
        }

        throw new InvalidOperationException("Rest pose did not converge.");
    }

    /// <summary>The elbow lock threshold: the largest elbow-tendon scale with a stable pose (shoulder tendon at scale 1).</summary>
    public ArmFold ElbowFold() => Fold(elbow: true);

    /// <summary>The shoulder lock threshold: the largest shoulder-tendon scale with a stable pose (elbow tendon at scale 1).</summary>
    public ArmFold ShoulderFold() => Fold(elbow: false);

    private double Tension(double length, double rest) =>
        length > rest ? g.AxialStiffness * (length - rest) / rest : 0.0;

    /// <summary>
    /// Follows the branch of rest poses from straight out, stepping the
    /// lengthened tendon's own joint angle (the lower angle for the elbow,
    /// the upper for the shoulder) and solving for the other angle and the
    /// scale. The scale peaks at the fold; the peak is refined by golden
    /// section.
    /// </summary>
    private ArmFold Fold(bool elbow)
    {
        // Branch point at a given driving angle a: unknowns (other angle b, scale s).
        (double Other, double Scale) Solve(double a, double b, double s)
        {
            for (var i = 0; i < 100; i++)
            {
                var f = Residual(a, b, s);
                const double h = 1e-7;
                var fb = Residual(a, b + h, s);
                var fs = Residual(a, b, s + h);
                double j11 = (fb.R1 - f.R1) / h, j21 = (fb.R2 - f.R2) / h;
                double j12 = (fs.R1 - f.R1) / h, j22 = (fs.R2 - f.R2) / h;
                var det = (j11 * j22) - (j12 * j21);
                var db = ((j22 * f.R1) - (j12 * f.R2)) / det;
                var ds = ((j11 * f.R2) - (j21 * f.R1)) / det;
                b -= db;
                s -= ds;
                if (Math.Abs(db) + Math.Abs(ds) < 1e-14)
                {
                    return (b, s);
                }
            }

            throw new InvalidOperationException("Branch point did not converge.");
        }

        (double R1, double R2) Residual(double a, double b, double s) =>
            elbow ? Gradient(b, a, 1, s) : Gradient(a, b, s, 1);

        // Walk down in 0.5° steps until the scale starts to fall.
        var step = -0.5 * Math.PI / 180;
        var points = new List<(double A, double B, double S)> { (0, 0, 1) };
        while (true)
        {
            var last = points[^1];
            var a = last.A + step;
            var (b, s) = Solve(a, last.B, last.S);
            points.Add((a, b, s));
            if (points.Count >= 3 && s < points[^2].S)
            {
                break;
            }

            if (a < -Math.PI / 2)
            {
                throw new InvalidOperationException("No fold before the link hangs straight down.");
            }
        }

        // The peak lies between points[^3] and points[^1]; golden section on the scale.
        var lo = points[^1].A;
        var hi = points[^3].A;
        var seed = points[^2];
        double ScaleAt(double a)
        {
            var (b, s) = Solve(a, seed.B, seed.S);
            return s;
        }

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
        var (other, scale) = Solve(peak, seed.B, seed.S);
        return elbow ? new ArmFold(scale, other, peak) : new ArmFold(scale, peak, other);
    }

    private static Vector2d Perp(Vector2d v) => new(-v.Y, v.X);
}
