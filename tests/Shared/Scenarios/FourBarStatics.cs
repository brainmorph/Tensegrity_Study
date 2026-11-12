using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The geometric prediction for the four-bar toggle clamp (PLAN.md M8),
/// written independently of the simulator and of the generator's
/// kinematics. With the crank at θ, the rocker at ψ, crank a, coupler b,
/// rocker c and ground d, the loop closes when Freudenstein's equation
/// <code>
/// F(θ, ψ) = K₁·cos ψ − K₂·cos θ + K₃ − cos(θ − ψ) = 0,
/// K₁ = d/a,  K₂ = d/c,  K₃ = (a² − b² + c² + d²) / (2·a·c)
/// </code>
/// holds. Differentiating it gives dψ/dθ = −F_θ / F_ψ, which is zero at
/// the toggle. By virtual work, the crank torque that balances a force f at
/// the rocker's end B is τ = −f·dB/dθ, with dB/dθ = c·(−sin ψ, cos ψ)·dψ/dθ.
///
/// The clamp's load is its band: anchored a band length toward −x from B
/// in the latched pose, stretched there by the band strain to the clamp
/// tension T₀, so EA = T₀ / strain and the rest length is length / (1 +
/// strain). Elsewhere it pulls B toward the anchor with EA·(L − L₀)/L₀, and
/// not at all once slack.
/// </summary>
public sealed class FourBarStatics(FourBarClampGeometry geometry)
{
    private readonly FourBarClampGeometry geometry = geometry;
    private readonly double c = geometry.RockerLength;
    private readonly double d = geometry.GroundLength;
    private readonly double k1 = geometry.GroundLength / geometry.CrankLength;
    private readonly double k2 = geometry.GroundLength / geometry.RockerLength;
    private readonly double k3 =
        ((geometry.CrankLength * geometry.CrankLength) - (geometry.CouplerLength * geometry.CouplerLength) + (geometry.RockerLength * geometry.RockerLength) + (geometry.GroundLength * geometry.GroundLength))
        / (2 * geometry.CrankLength * geometry.RockerLength);

    public FourBarStatics()
        : this(new FourBarClampGeometry())
    {
    }

    /// <summary>The rocker angle that closes the loop with the crank at θ, by Newton's method from <paramref name="guess"/>, rad.</summary>
    public double RockerAngle(double crank, double guess)
    {
        var psi = guess;
        for (var i = 0; i < 100; i++)
        {
            var step = Closure(crank, psi) / ClosureByRocker(crank, psi);
            psi -= step;
            if (Math.Abs(step) < 1e-15)
            {
                return psi;
            }
        }

        throw new InvalidOperationException("The loop did not close.");
    }

    /// <summary>dψ/dθ: how fast the rocker turns per unit crank turn.</summary>
    public double Ratio(double crank, double rocker) => -ClosureByCrank(crank, rocker) / ClosureByRocker(crank, rocker);

    /// <summary>
    /// The toggle: the crank and rocker angles where dψ/dθ = 0, by Newton's
    /// method on the loop closure and F_θ = 0 together, from a guess.
    /// </summary>
    public (double Crank, double Rocker) Toggle(double crankGuess, double rockerGuess)
    {
        double theta = crankGuess, psi = rockerGuess;
        for (var i = 0; i < 100; i++)
        {
            // Residuals: closure F and its crank derivative F_θ; Jacobian by hand.
            var r1 = Closure(theta, psi);
            var r2 = ClosureByCrank(theta, psi);
            var j11 = ClosureByCrank(theta, psi);
            var j12 = ClosureByRocker(theta, psi);
            var j21 = (k2 * Math.Cos(theta)) + Math.Cos(theta - psi);
            var j22 = -Math.Cos(theta - psi);
            var det = (j11 * j22) - (j12 * j21);
            var dTheta = ((r1 * j22) - (j12 * r2)) / det;
            var dPsi = ((j11 * r2) - (r1 * j21)) / det;
            theta -= dTheta;
            psi -= dPsi;
            if (Math.Abs(dTheta) + Math.Abs(dPsi) < 1e-15)
            {
                return (theta, psi);
            }
        }

        throw new InvalidOperationException("The toggle did not converge.");
    }

    /// <summary>The crank torque that holds the clamp still against a force (fx, fy) at the rocker's end B, N·m.</summary>
    public double HoldingTorque(double crank, double rocker, double fx, double fy)
    {
        var rate = Ratio(crank, rocker);
        var dbx = -c * Math.Sin(rocker) * rate;
        var dby = c * Math.Cos(rocker) * rate;
        return -((fx * dbx) + (fy * dby));
    }

    /// <summary>The crank torque that holds the clamp still against its band, which carries <paramref name="clampTension"/> when latched, N·m.</summary>
    public double BandHoldingTorque(double crank, double rocker, double clampTension)
    {
        var (fx, fy) = BandForce(rocker, clampTension);
        return HoldingTorque(crank, rocker, fx, fy);
    }

    /// <summary>The band's pull on B with the rocker at ψ, N.</summary>
    public (double Fx, double Fy) BandForce(double rocker, double clampTension)
    {
        var (bx, by) = EndB(rocker);
        var (ax, ay) = Anchor();
        var length = Math.Sqrt(((ax - bx) * (ax - bx)) + ((ay - by) * (ay - by)));
        var rest = RestLength();
        if (!(length > rest))
        {
            return (0, 0);
        }

        var tension = clampTension / geometry.BandStrain * (length - rest) / rest;
        return (tension * (ax - bx) / length, tension * (ay - by) / length);
    }

    /// <summary>Elastic energy in the band with the rocker at ψ, J.</summary>
    public double BandEnergy(double rocker, double clampTension)
    {
        var (bx, by) = EndB(rocker);
        var (ax, ay) = Anchor();
        var stretch = Math.Max(0, Math.Sqrt(((ax - bx) * (ax - bx)) + ((ay - by) * (ay - by))) - RestLength());
        return 0.5 * (clampTension / geometry.BandStrain / RestLength()) * stretch * stretch;
    }

    /// <summary>The rocker's end B with the rocker at ψ, m.</summary>
    public (double X, double Y) EndB(double rocker) => (d + (c * Math.Cos(rocker)), c * Math.Sin(rocker));

    /// <summary>The latched crank and rocker angles: the toggle plus the over-center angle.</summary>
    public (double Crank, double Rocker) Latched()
    {
        var (crank, rocker) = Toggle(0.5, 1.6);
        var latched = crank + geometry.OverCenter;
        return (latched, RockerAngle(latched, rocker));
    }

    private (double X, double Y) Anchor()
    {
        var (_, rocker) = Latched();
        var (bx, by) = EndB(rocker);
        return (bx - geometry.BandLength, by);
    }

    private double RestLength() => geometry.BandLength / (1 + geometry.BandStrain);

    private double Closure(double theta, double psi) => (k1 * Math.Cos(psi)) - (k2 * Math.Cos(theta)) + k3 - Math.Cos(theta - psi);

    private double ClosureByCrank(double theta, double psi) => (k2 * Math.Sin(theta)) + Math.Sin(theta - psi);

    private double ClosureByRocker(double theta, double psi) => (-k1 * Math.Sin(psi)) - Math.Sin(theta - psi);
}
