using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Elements;

/// <summary>Test 1 (PLAN.md section 7): the cable force law on its own.</summary>
public class CableTests
{
    // L0 = 2 m, EA = 1000 N, so k = 500 N/m; c = 10 N·s/m.
    private static readonly Cable Damped = new(
        "c", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("l", Vector2d.Zero), 2.0, 1000.0, 10.0);

    private static readonly Cable Undamped = new(
        "u", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("l", Vector2d.Zero), 2.0, 1000.0);

    [Fact]
    public void StiffnessIsAxialStiffnessOverRestLength() => Assert.Equal(500.0, Damped.Stiffness);

    [Fact]
    public void TautTensionIsSpringPlusDamper()
    {
        Assert.Equal(500.0 * 0.1, Undamped.Tension(2.1, 0.0), 12);
        Assert.Equal((500.0 * 0.1) + (10.0 * 0.3), Damped.Tension(2.1, 0.3), 12);
        Assert.Equal(CableMode.Taut, Damped.ModeAt(2.1, 0.3));
    }

    [Fact]
    public void DamperRaisesTensionWhenLengtheningAndLowersItWhenShortening()
    {
        var still = Damped.Tension(2.1, 0.0);

        Assert.True(Damped.Tension(2.1, 0.5) > still);
        Assert.True(Damped.Tension(2.1, -0.5) < still);
    }

    [Theory]
    [InlineData(1.9, 0.0)]
    [InlineData(1.9, -3.0)]
    [InlineData(1.9, 100.0)] // Lengthening fast while slack still exerts no force.
    [InlineData(1.999999, 1e6)]
    public void SlackCableExertsNoForce(double length, double rate)
    {
        Assert.Equal(CableMode.Slack, Damped.ModeAt(length, rate));
        Assert.Equal(0.0, Damped.Tension(length, rate));
    }

    [Fact]
    public void RecoilingCableIsStretchedButExertsNoForce()
    {
        // k·Δ + c·L̇ = 500·0.1 + 10·(−6) = −10 < 0: the damper would push, which a cable cannot.
        Assert.Equal(CableMode.Recoiling, Damped.ModeAt(2.1, -6.0));
        Assert.Equal(0.0, Damped.Tension(2.1, -6.0));
    }

    [Fact]
    public void TensionIsNeverNegative()
    {
        for (var length = 1.5; length <= 2.5; length += 0.01)
        {
            for (var rate = -20.0; rate <= 20.0; rate += 0.5)
            {
                Assert.True(Damped.Tension(length, rate) >= 0, $"Negative tension at L = {length}, L̇ = {rate}.");
            }
        }
    }

    [Fact]
    public void AtRestLengthTheCableIsTautWithOnlyTheDamperForce()
    {
        Assert.Equal(CableMode.Taut, Damped.ModeAt(2.0, 0.0));
        Assert.Equal(0.0, Damped.Tension(2.0, 0.0));
        Assert.Equal(10.0 * 0.2, Damped.Tension(2.0, 0.2), 15);
        Assert.Equal(CableMode.Recoiling, Damped.ModeAt(2.0, -0.2));
    }

    [Fact]
    public void LatchedTautFormulaIsUnclampedSoItStaysSmooth()
    {
        // Inside a latched step the taut formula continues past the switch.
        Assert.Equal(500.0 * -0.1, Undamped.TensionInMode(CableMode.Taut, 1.9, 0.0), 12);
        Assert.Equal(0.0, Undamped.TensionInMode(CableMode.Slack, 2.5, 0.0));
        Assert.Equal(0.0, Undamped.TensionInMode(CableMode.Recoiling, 2.5, 0.0));
    }

    [Fact]
    public void RestLengthForGivesTheRequestedTensionAtRest()
    {
        var restLength = Cable.RestLengthFor(length: 1.3, tension: 40.0, axialStiffness: 1000.0);
        var cable = new Cable("c", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("l", Vector2d.Zero), restLength, 1000.0);

        Assert.Equal(40.0, cable.Tension(1.3, 0.0), 12);
    }

    [Fact]
    public void RejectsBadCables()
    {
        var ground = Attachment.Ground(Vector2d.Zero);
        var onL = Attachment.OnLink("l", Vector2d.Zero);

        Assert.Throws<ArgumentException>(() => new Cable("c", ground, ground, 1, 1));
        Assert.Throws<ArgumentException>(() => new Cable("c", onL, Attachment.OnLink("l", Vector2d.UnitX), 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Cable("c", ground, onL, 0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Cable("c", ground, onL, 1, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Cable("c", ground, onL, 1, 1, -0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Cable("c", ground, onL, double.NaN, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => Cable.RestLengthFor(1, -1, 1));
    }
}
