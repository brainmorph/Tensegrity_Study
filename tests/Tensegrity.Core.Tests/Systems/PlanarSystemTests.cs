using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Systems;

public class PlanarSystemTests
{
    private static PlanarSystem TwoBodies() =>
        new([PlanarLink.SlenderBar("a", 1.0, 1.0), PlanarLink.SlenderBar("b", 2.0, 0.5)], PlanarSystem.StandardGravity);

    [Fact]
    public void StateLayoutIsPositionsThenVelocitiesThenDissipatedEnergyThenActuatorAndExternalWork()
    {
        var system = TwoBodies();

        Assert.Equal(15, system.StateLength);
        Assert.Equal(12, system.DissipatedEnergyIndex);
        Assert.Equal(13, system.ActuatorWorkIndex);
        Assert.Equal(14, system.ExternalWorkIndex);
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
        Assert.Equal([0, -9.81, 0, 0, -9.81, 0], dy[6..12]);
        Assert.Equal(0.0, dy[12]); // Nothing dissipates without cables.
        Assert.Equal(0.0, dy[13]); // No actuators, no work.
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

        Assert.Throws<ArgumentException>(() => system.Evaluate(0, new double[6], new double[13]));
        Assert.Throws<ArgumentException>(() => system.GetPose(new double[6], 0));
    }

    [Fact]
    public void RejectsBadConstruction()
    {
        var a = PlanarLink.SlenderBar("a", 1.0, 1.0);

        Assert.Throws<ArgumentException>(() => new PlanarSystem([], Vector2d.Zero));
        Assert.Throws<ArgumentException>(() => new PlanarSystem([a, PlanarLink.SlenderBar("a", 2, 2)], Vector2d.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PlanarSystem([a], new Vector2d(double.NaN, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => TwoBodies().GetPose(new double[TwoBodies().StateLength], 2));
    }

    [Fact]
    public void RejectsBadPins()
    {
        var a = PlanarLink.SlenderBar("a", 1.0, 1.0);
        var ground = Attachment.Ground(Vector2d.Zero);
        var onA = Attachment.OnLink("a", a.EndA);

        Assert.Throws<ArgumentException>(() => new PinJoint("p", ground, ground));
        Assert.Throws<ArgumentException>(() => new PinJoint("p", onA, Attachment.OnLink("a", a.EndB)));
        Assert.Throws<ArgumentException>(
            () => new PlanarSystem([a], Vector2d.Zero, [new PinJoint("p", Attachment.OnLink("missing", Vector2d.Zero), ground)]));
        Assert.Throws<ArgumentException>(
            () => new PlanarSystem([a], Vector2d.Zero, [new PinJoint("p", onA, ground), new PinJoint("p", onA, ground)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConstraintStabilization(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => Attachment.Ground(new Vector2d(double.NaN, 0)));
    }

    [Fact]
    public void WorldPointAndVelocityFollowTheBody()
    {
        var system = TwoBodies();
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(1, 2), Math.PI / 2), new Twist2d(new Vector2d(3, 0), 2));
        var end = Attachment.OnLink("a", new Vector2d(0.5, 0));

        // Local (0.5, 0) rotated 90° is (0, 0.5); its velocity adds ω × r = 2·(−0.5, 0).
        Assert.Equal(1.0, system.WorldPoint(state.Values, end).X, 15);
        Assert.Equal(2.5, system.WorldPoint(state.Values, end).Y, 15);
        Assert.Equal(2.0, system.PointVelocity(state.Values, end).X, 15);
        Assert.Equal(0.0, system.PointVelocity(state.Values, end).Y, 15);
        Assert.Equal(new Vector2d(7, 8), system.WorldPoint(state.Values, Attachment.Ground(new Vector2d(7, 8))));
    }
}
