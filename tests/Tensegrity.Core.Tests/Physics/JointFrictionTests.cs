using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Viscous joint friction (PLAN.md 4.3): a couple −c·Δω across a pin whose
/// loss c·Δω² enters the dissipated energy.
/// </summary>
public class JointFrictionTests
{
    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);
    private static readonly PlanarLink Upper = PlanarLink.SlenderBar("upper", mass: 1.0, length: 1.0);
    private static readonly PlanarLink Lower = PlanarLink.SlenderBar("lower", mass: 0.6, length: 0.8);

    private static PlanarSystem PinnedBar(Vector2d gravity, double friction) => new(
        [Bar],
        gravity,
        [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero), friction)]);

    private static SystemState PinnedBarState(PlanarSystem system, double theta, double omega)
    {
        var state = system.CreateState();
        var r = (-Bar.EndA).Rotated(theta);
        system.SetBody(state.Values, 0, new Pose2d(r, theta), new Twist2d(Vector2d.Cross(omega, r), omega));
        return state;
    }

    [Fact]
    public void FrictionSpinsDownAFreeRotorExponentially()
    {
        const double c = 0.3;
        const double omega0 = 5.0;
        var inertia = Bar.MomentOfInertiaAbout(Bar.EndA);
        var system = PinnedBar(Vector2d.Zero, c);
        var state = PinnedBarState(system, 0.0, omega0);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 2000);

        // I·ω̇ = −c·ω, so ω(t) = ω0·exp(−c·t/I).
        var omega = omega0 * Math.Exp(-c * 2.0 / inertia);
        Assert.Equal(omega, system.GetTwist(state.Values, 0).Angular, 9);

        // Measured 1.0e-8 J off of 8.56 J, falling 16x per dt halving.
        Assert.Equal(0.5 * inertia * ((omega0 * omega0) - (omega * omega)), system.DissipatedEnergy(state.Values), 1e-7);
    }

    [Fact]
    public void FrictionalPendulumEnergyBalanceCloses()
    {
        var system = PinnedBar(PlanarSystem.StandardGravity, 0.2);
        var state = PinnedBarState(system, -Math.PI / 2 + 2.5, 0.0);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;
        var previous = 0.0;
        var monotone = true;

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, 50_000, (_, s) =>
        {
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial));
            var dissipated = system.DissipatedEnergy(s.Values);
            monotone &= dissipated >= previous;
            previous = dissipated;
        });

        Assert.True(monotone, "Dissipated energy must never decrease.");
        Assert.True(system.DissipatedEnergy(state.Values) > 5, $"Only {system.DissipatedEnergy(state.Values)} J dissipated.");
        Assert.InRange(drift, 0.0, 1e-10);
    }

    [Fact]
    public void ElbowFrictionIsInternalSoAFreeChainKeepsItsMomentum()
    {
        var system = new PlanarSystem(
            [Upper, Lower],
            Vector2d.Zero,
            [new PinJoint("elbow", Attachment.OnLink("lower", Lower.EndA), Attachment.OnLink("upper", Upper.EndB), friction: 0.05)]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(Vector2d.Zero, 0), new Twist2d(Vector2d.Zero, 0));

        // Lower spins at 3 rad/s about the elbow, which is at rest.
        var r = -Lower.EndA;
        system.SetBody(state.Values, 1, new Pose2d(Upper.EndB + r, 0), new Twist2d(Vector2d.Cross(3.0, r), 3.0));
        var momentum = system.AngularMomentum(state.Values, Vector2d.Zero);
        var energy = system.TotalEnergy(state.Values);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 3000);

        var relative = system.GetTwist(state.Values, 1).Angular - system.GetTwist(state.Values, 0).Angular;
        Assert.InRange(Math.Abs(relative), 0.0, 1.0);
        Assert.Equal(momentum, system.AngularMomentum(state.Values, Vector2d.Zero), 9);
        Assert.Equal(energy, system.TotalEnergy(state.Values), 9);
    }

    [Fact]
    public void RejectsBadFriction()
    {
        var a = Attachment.OnLink("bar", Bar.EndA);
        var ground = Attachment.Ground(Vector2d.Zero);

        Assert.Equal(0.0, new PinJoint("p", a, ground).Friction);
        Assert.Throws<ArgumentOutOfRangeException>(() => new PinJoint("p", a, ground, -0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PinJoint("p", a, ground, double.NaN));
    }
}
