using Tensegrity.Core.Elements;

namespace Tensegrity.Core.Tests.Elements;

/// <summary>
/// The one-sided contact law and the joint limit built from it: stops push
/// the joint angle back into range and never pull it out.
/// </summary>
public class JointLimitTests
{
    private static readonly ContactLaw Law = new(stiffness: 200.0, damping: 4.0);
    private static readonly JointLimit Elbow = new(lower: -2.5, upper: 0.1, stiffness: 200.0, damping: 4.0);

    [Fact]
    public void ContactPushesOnlyWhileOverlapping()
    {
        Assert.Equal(ContactMode.Clear, Law.ModeAt(-0.01, 50.0));
        Assert.Equal(0.0, Law.Force(-0.01, 50.0));

        Assert.Equal(ContactMode.Pressing, Law.ModeAt(0.02, -0.5));
        Assert.Equal((200.0 * 0.02) + (4.0 * -0.5), Law.Force(0.02, -0.5), 12);

        // Separating faster than the spring can follow: no pull.
        Assert.Equal(ContactMode.Recoiling, Law.ModeAt(0.02, -2.0));
        Assert.Equal(0.0, Law.Force(0.02, -2.0));

        // Touching at δ = 0 counts as contact, so the switch is at the boundary.
        Assert.Equal(ContactMode.Pressing, Law.ModeAt(0.0, 0.0));
        Assert.Equal(ContactMode.Recoiling, Law.ModeAt(0.0, -1e-9));
    }

    [Fact]
    public void LatchedPressingLawIsSmoothPastTheSwitch()
    {
        // An RK4 stage that strays past a switch still sees the unclamped law.
        Assert.Equal((200.0 * -0.01) + (4.0 * 1.0), Law.ForceInMode(ContactMode.Pressing, -0.01, 1.0), 12);
        Assert.Equal(0.0, Law.ForceInMode(ContactMode.Recoiling, 0.02, 0.0));
        Assert.Equal(0.0, Law.ForceInMode(ContactMode.Clear, 0.02, 0.0));
    }

    [Fact]
    public void ContactEnergyAndDissipation()
    {
        Assert.Equal(0.5 * 200.0 * 0.03 * 0.03, Law.ElasticEnergy(0.03), 15);
        Assert.Equal(0.0, Law.ElasticEnergy(-0.03));
        Assert.Equal(4.0 * 0.25, Law.DissipatedPowerInMode(ContactMode.Pressing, 0.01, -0.5), 15);
        Assert.Equal(-200.0 * 0.02 * -2.0, Law.DissipatedPowerInMode(ContactMode.Recoiling, 0.02, -2.0), 12);
        Assert.Equal(0.0, Law.DissipatedPowerInMode(ContactMode.Clear, -0.02, 2.0));
    }

    [Fact]
    public void StopsPushTheJointAngleBackIntoRange()
    {
        Assert.Equal(0.0, Elbow.Torque(-1.0, 3.0));
        Assert.Equal((ContactMode.Clear, ContactMode.Clear), (Elbow.LowerMode(-1.0, 3.0), Elbow.UpperMode(-1.0, 3.0)));

        // 0.05 rad past the lower stop and still closing at 1 rad/s: δ̇ = +1.
        Assert.Equal((200.0 * 0.05) + (4.0 * 1.0), Elbow.Torque(-2.55, -1.0), 12);
        Assert.Equal(ContactMode.Pressing, Elbow.LowerMode(-2.55, -1.0));

        // 0.05 rad past the upper stop and closing: the torque on A is negative.
        Assert.Equal(-((200.0 * 0.05) + (4.0 * 1.0)), Elbow.Torque(0.15, 1.0), 12);
        Assert.Equal(ContactMode.Pressing, Elbow.UpperMode(0.15, 1.0));

        // Leaving the upper stop fast: recoiling, no pull.
        Assert.Equal(ContactMode.Recoiling, Elbow.UpperMode(0.15, -5.0));
        Assert.Equal(0.0, Elbow.Torque(0.15, -5.0));

        Assert.Equal(0.5 * 200.0 * 0.05 * 0.05, Elbow.ElasticEnergy(0.15), 12);
        Assert.Equal(0.5 * 200.0 * 0.05 * 0.05, Elbow.ElasticEnergy(-2.55), 12);
        Assert.Equal(0.0, Elbow.ElasticEnergy(0.0));
    }

    [Fact]
    public void OneSidedLimitHasNoOtherStop()
    {
        var hyperextension = new JointLimit(double.NegativeInfinity, 0.0, 100.0);

        Assert.False(hyperextension.HasLower);
        Assert.True(hyperextension.HasUpper);
        Assert.Equal(ContactMode.Clear, hyperextension.LowerMode(-1e6, -1.0));
        Assert.Equal(0.0, hyperextension.Torque(-1e6, -1.0));
        Assert.Equal(0.0, hyperextension.ElasticEnergy(-1e6));
        Assert.Equal(-100.0 * 0.1, hyperextension.Torque(0.1, 0.0), 12);
    }

    [Fact]
    public void RejectsBadLimits()
    {
        Assert.Throws<ArgumentException>(() => new JointLimit(0.5, 0.5, 100.0));
        Assert.Throws<ArgumentException>(() => new JointLimit(0.5, -0.5, 100.0));
        Assert.Throws<ArgumentException>(() => new JointLimit(double.NegativeInfinity, double.PositiveInfinity, 100.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointLimit(double.NaN, 0.5, 100.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointLimit(double.PositiveInfinity, 0.5, 100.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointLimit(-0.5, double.NegativeInfinity, 100.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointLimit(-0.5, 0.5, 0.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointLimit(-0.5, 0.5, 100.0, -1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ContactLaw(double.PositiveInfinity));
    }
}
