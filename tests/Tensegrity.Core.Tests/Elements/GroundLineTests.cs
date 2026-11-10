using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Elements;

/// <summary>
/// The ground line's friction law: a spring-damper in series with a
/// Coulomb slider. Stick and slip are decided by the state alone, and the
/// force is continuous at both switches.
/// </summary>
public class GroundLineTests
{
    private const double Kt = 1e4;
    private const double Ct = 60.0;
    private const double Mu = 0.5;

    private static readonly GroundLine Line = new(
        [new ContactPoint("foot", "bar", new Vector2d(0.5, 0))],
        normalStiffness: 1e4,
        normalDamping: 60.0,
        friction: Mu,
        tangentialStiffness: Kt,
        tangentialDamping: Ct,
        height: 0.25);

    [Fact]
    public void PenetrationIsMeasuredDownFromTheLine()
    {
        Assert.Equal(0.05, Line.Penetration(0.2), 15);
        Assert.Equal(-0.75, Line.Penetration(1.0), 15);
    }

    [Fact]
    public void OffTheGroundThereIsNoFrictionAndTheSpringRelaxes()
    {
        Assert.Equal(FrictionMode.Free, Line.FrictionModeAt(ContactMode.Clear, 0.0, 1e-3, 5.0));
        Assert.Equal(FrictionMode.Free, Line.FrictionModeAt(ContactMode.Recoiling, 0.0, 1e-3, 5.0));
        Assert.Equal(0.0, Line.FrictionForce(FrictionMode.Free, 0.0, 1e-3, 5.0));
        Assert.Equal(-Kt * 1e-3 / Ct, Line.ExtensionRate(FrictionMode.Free, 0.0, 1e-3, 5.0), 15);
    }

    [Fact]
    public void SticksInsideTheFrictionConeAndSlipsOutside()
    {
        const double n = 10.0;

        // Held force k_t·z + c_t·v against μ·N = 5 N.
        Assert.Equal(FrictionMode.Stick, Line.FrictionModeAt(ContactMode.Pressing, n, 4e-4, 0.0));
        Assert.Equal(-4.0, Line.FrictionForce(FrictionMode.Stick, n, 4e-4, 0.0), 12);
        Assert.Equal(0.02, Line.ExtensionRate(FrictionMode.Stick, n, 4e-4, 0.02));

        Assert.Equal(FrictionMode.SlipPositive, Line.FrictionModeAt(ContactMode.Pressing, n, 4e-4, 0.02));
        Assert.Equal(-Mu * n, Line.FrictionForce(FrictionMode.SlipPositive, n, 4e-4, 0.02));
        Assert.Equal(FrictionMode.SlipNegative, Line.FrictionModeAt(ContactMode.Pressing, n, -4e-4, -0.02));
        Assert.Equal(Mu * n, Line.FrictionForce(FrictionMode.SlipNegative, n, -4e-4, -0.02));

        // Exactly on the cone counts as stick.
        Assert.Equal(FrictionMode.Stick, Line.FrictionModeAt(ContactMode.Pressing, n, 5e-4, 0.0));
    }

    [Fact]
    public void ForceIsContinuousWhereStickTurnsToSlip()
    {
        // On the cone boundary, k_t·z + c_t·v = μ·N.
        const double n = 10.0;
        const double v = 0.03;
        var z = ((Mu * n) - (Ct * v)) / Kt;

        Assert.Equal(
            Line.FrictionForce(FrictionMode.Stick, n, z, v),
            Line.FrictionForce(FrictionMode.SlipPositive, n, z, v),
            12);
        Assert.Equal(
            Line.ExtensionRate(FrictionMode.Stick, n, z, v),
            Line.ExtensionRate(FrictionMode.SlipPositive, n, z, v),
            12);
    }

    [Fact]
    public void SliderSpeedIsZeroWhereSlipTurnsToStick()
    {
        // In slip the slider moves at u = v − ż; it stops exactly where the stick law would carry μ·N.
        const double n = 10.0;
        const double z = 3e-4;
        var v = ((Mu * n) - (Kt * z)) / Ct;

        var u = v - Line.ExtensionRate(FrictionMode.SlipPositive, n, z, v);
        Assert.Equal(0.0, u, 15);
        Assert.Equal(-Mu * n, Line.FrictionForce(FrictionMode.Stick, n, z, v), 12);
    }

    [Theory]
    [InlineData(FrictionMode.Stick, 2e-4, 0.01)]
    [InlineData(FrictionMode.SlipPositive, 1e-4, 0.5)]
    [InlineData(FrictionMode.SlipNegative, -1e-4, -0.5)]
    [InlineData(FrictionMode.Free, 1e-3, 0.5)]
    public void PowerBalancesWithStoredAndDissipatedEnergy(FrictionMode mode, double z, double v)
    {
        // Power the ground takes from the point, −F·v, goes into the spring (k_t·z·ż) and the losses.
        const double n = 10.0;
        var force = Line.FrictionForce(mode, n, z, v);
        var rate = Line.ExtensionRate(mode, n, z, v);
        var stored = Kt * z * rate;
        var lost = Line.TangentialDissipatedPower(mode, n, z, v);

        Assert.True(lost >= 0, $"Dissipation {lost} must not be negative.");
        Assert.Equal(-force * v, stored + lost, 12);
    }

    [Fact]
    public void RejectsBadGrounds()
    {
        ContactPoint[] one = [new("a", "bar", Vector2d.Zero)];
        Assert.Throws<ArgumentException>(() => new GroundLine([], 1e4, 0, 0.5, 1e4, 60));
        Assert.Throws<ArgumentException>(() => new GroundLine([one[0], new("a", "rod", Vector2d.Zero)], 1e4, 0, 0.5, 1e4, 60));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GroundLine(one, 1e4, 0, -0.1, 1e4, 60));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GroundLine(one, 1e4, 0, 0.5, 0, 60));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GroundLine(one, 1e4, 0, 0.5, 1e4, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GroundLine(one, 0, 0, 0.5, 1e4, 60));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GroundLine(one, 1e4, 0, 0.5, 1e4, 60, double.NaN));
    }
}
