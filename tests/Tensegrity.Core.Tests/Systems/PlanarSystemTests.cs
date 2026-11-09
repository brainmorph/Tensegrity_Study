using Tensegrity.Core.Bodies;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Systems;

public class PlanarSystemTests
{
    private static PlanarSystem TwoBodies() =>
        new([PlanarLink.SlenderBar("a", 1.0, 1.0), PlanarLink.SlenderBar("b", 2.0, 0.5)], PlanarSystem.StandardGravity);

    [Fact]
    public void StateLayoutIsPositionsThenVelocities()
    {
        var system = TwoBodies();

        Assert.Equal(12, system.StateLength);
        Assert.Equal(0, system.PositionIndex(0));
        Assert.Equal(3, system.PositionIndex(1));
        Assert.Equal(6, system.VelocityIndex(0));
        Assert.Equal(9, system.VelocityIndex(1));
    }

    [Fact]
    public void SetBodyRoundTripsThroughGetters()
    {
        var system = TwoBodies();
        var state = system.CreateState();
        var pose = new Pose2d(new Vector2d(1, 2), 3);
        var twist = new Twist2d(new Vector2d(4, 5), 6);

        system.SetBody(state.Values, 1, pose, twist);

        Assert.Equal(pose, system.GetPose(state.Values, 1));
        Assert.Equal(twist, system.GetTwist(state.Values, 1));
        Assert.Equal(default, system.GetPose(state.Values, 0));
    }

    [Fact]
    public void EvaluateGivesGravitationalAccelerationAndNoTorque()
    {
        var system = TwoBodies();
        var state = system.CreateState();
        system.SetBody(state.Values, 1, new Pose2d(Vector2d.Zero, 0.5), new Twist2d(new Vector2d(7, 8), 9));
        var dy = new double[system.StateLength];

        system.Evaluate(0, state.Values, dy);

        Assert.Equal([0, 0, 0, 7, 8, 9], dy[..6]);
        Assert.Equal([0, -9.81, 0, 0, -9.81, 0], dy[6..]);
    }

    [Fact]
    public void AngularMomentumOfTranslatingBodyIsRCrossMv()
    {
        var system = new PlanarSystem([PlanarLink.SlenderBar("a", 2.0, 1.0)], Vector2d.Zero);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, 3), 0), new Twist2d(new Vector2d(1, 0), 0));

        // r × m·v = (0, 3) × (2, 0) = −6.
        Assert.Equal(-6.0, system.AngularMomentum(state.Values, Vector2d.Zero));
    }

    [Fact]
    public void RejectsWrongStateLength()
    {
        var system = TwoBodies();

        Assert.Throws<ArgumentException>(() => system.Evaluate(0, new double[6], new double[12]));
        Assert.Throws<ArgumentException>(() => system.GetPose(new double[6], 0));
    }

    [Fact]
    public void RejectsBadConstruction()
    {
        var a = PlanarLink.SlenderBar("a", 1.0, 1.0);

        Assert.Throws<ArgumentException>(() => new PlanarSystem([], Vector2d.Zero));
        Assert.Throws<ArgumentException>(() => new PlanarSystem([a, PlanarLink.SlenderBar("a", 2, 2)], Vector2d.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PlanarSystem([a], new Vector2d(double.NaN, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => TwoBodies().GetPose(new double[12], 2));
    }
}
