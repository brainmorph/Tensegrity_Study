using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Global settling damping (PLAN.md 4.3): −γ·m·v and −γ·I·ω on every body,
/// off by default, its loss counted as dissipated energy.
/// </summary>
public class GlobalDampingTests
{
    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);

    [Fact]
    public void FreeBodySlowsExponentially()
    {
        const double gamma = 1.5;
        var system = new PlanarSystem([Bar], Vector2d.Zero, globalDamping: gamma);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(Vector2d.Zero, 0), new Twist2d(new Vector2d(2.0, -1.0), 3.0));
        var energy = system.KineticEnergy(state.Values);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 1_000);

        // v(t) = v0·e^(−γt); x(t) = v0·(1 − e^(−γt))/γ.
        var decay = Math.Exp(-gamma * 1.0);
        var twist = system.GetTwist(state.Values, 0);
        Assert.Equal(2.0 * decay, twist.Linear.X, 11);
        Assert.Equal(-1.0 * decay, twist.Linear.Y, 11);
        Assert.Equal(3.0 * decay, twist.Angular, 11);
        Assert.Equal(2.0 * (1 - decay) / gamma, system.GetPose(state.Values, 0).Position.X, 11);
        Assert.Equal(energy * (1 - (decay * decay)), system.DissipatedEnergy(state.Values), 10);
    }

    [Fact]
    public void PinnedBarSettlesHangingWithTheEnergyBalanceClosed()
    {
        var system = new PlanarSystem(
            [Bar],
            PlanarSystem.StandardGravity,
            [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero))],
            globalDamping: 6.0);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d((-Bar.EndA).Rotated(0.3), 0.3), default);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, 100_000, (_, s) => drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        // The damping torque about the pivot is −γ·I_pivot·θ̇, so the swing decays as e^(−γt/2) = e^(−30).
        Assert.Equal(-Math.PI / 2, system.GetPose(state.Values, 0).Angle, 9);
        Assert.InRange(system.KineticEnergy(state.Values), 0.0, 1e-15);
        Assert.InRange(drift, 0.0, 1e-10);
    }

    [Fact]
    public void OffByDefaultAndRejectsBadValues()
    {
        Assert.Equal(0.0, new PlanarSystem([Bar], Vector2d.Zero).GlobalDamping);
        Assert.Throws<ArgumentOutOfRangeException>(() => new PlanarSystem([Bar], Vector2d.Zero, globalDamping: -0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PlanarSystem([Bar], Vector2d.Zero, globalDamping: double.NaN));
    }
}
