using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// External loads (PLAN.md M6 perturbations): a world-frame force at a
/// body point, piecewise-linear in time. Checked against momentum and
/// angular-impulse laws, the energy balance with external work, and RK4
/// order across the schedule's breakpoints.
/// </summary>
public class ExternalLoadTests
{
    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);

    private static EventSplittingStepper Stepper(PlanarSystem system) => new(new Rk4Integrator(), system);

    [Fact]
    public void ConstantForceAtTheCenterAcceleratesAFreeBodyAndDoesWork()
    {
        var force = new Vector2d(3.0, -1.0);
        var system = new PlanarSystem([Bar], Vector2d.Zero, loads: [new ExternalLoad("push", "bar", Vector2d.Zero, [new LoadPoint(0, force.X, force.Y)])]);
        var state = system.CreateState();

        FixedStepRunner.Run(state, Stepper(system), 1e-3, 2_000);

        // x = ½·(F/m)·t², no rotation, and the work is F·x.
        var x = 0.5 * (force / Bar.Mass) * 4.0;
        var pose = system.GetPose(state.Values, 0);
        Assert.Equal(x.X, pose.Position.X, 12);
        Assert.Equal(x.Y, pose.Position.Y, 12);
        Assert.Equal(0.0, pose.Angle, 15);
        Assert.Equal(Vector2d.Dot(force, x), system.ExternalWork(state.Values), 11);
        Assert.Equal(system.KineticEnergy(state.Values), system.ExternalWork(state.Values), 11);
    }

    [Fact]
    public void PulseDeliversItsImpulseToAFreeBar()
    {
        // A 2 ms pulse at the bar's end: linear momentum changes by exactly J,
        // angular momentum about the center by r × J (r turns slightly during the pulse).
        var impulse = new Vector2d(0.0, 0.5);
        var system = new PlanarSystem([Bar], Vector2d.Zero, loads: [ExternalLoad.Pulse("tap", "bar", Bar.EndB, impulse, start: 0.01, duration: 2e-3)]);
        var state = system.CreateState();
        var stepper = Stepper(system);

        FixedStepRunner.Run(state, stepper, 1e-4, 500);

        var twist = system.GetTwist(state.Values, 0);
        Assert.Equal(impulse.X / Bar.Mass, twist.Linear.X, 12);
        Assert.Equal(impulse.Y / Bar.Mass, twist.Linear.Y, 12);
        var spin = Vector2d.Cross(Bar.EndB, impulse) / Bar.MomentOfInertia;
        Assert.Equal(spin, twist.Angular, 3);
        Assert.Equal(3, stepper.EventCount);
        Assert.Equal(system.KineticEnergy(state.Values), system.ExternalWork(state.Values), 10);
    }

    [Fact]
    public void PulseOnAPinnedBarGivesTheAngularImpulseAboutThePin()
    {
        var impulse = new Vector2d(0.0, -0.4);
        var system = new PlanarSystem(
            [Bar],
            Vector2d.Zero,
            [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero))],
            loads: [ExternalLoad.Pulse("tap", "bar", Bar.EndB, impulse, start: 0.0, duration: 1e-3)]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(-Bar.EndA, 0), default);

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 100);

        // I_pivot·ω = ℓ × J, to within the bar's turn during the 1 ms pulse.
        var expected = Vector2d.Cross(Bar.EndB - Bar.EndA, impulse) / Bar.MomentOfInertiaAbout(Bar.EndA);
        Assert.Equal(expected, system.GetTwist(state.Values, 0).Angular, 4);
        // The 800 N peak opens the pin by a few 1e-11 m, which stabilization then closes.
        Assert.InRange(system.PinPositionResidual(state.Values), 0.0, 1e-10);
        Assert.Equal(system.KineticEnergy(state.Values), system.ExternalWork(state.Values), 10);
    }

    [Fact]
    public void EnergyBalanceClosesWithAForceHistoryOnAPendulum()
    {
        var system = new PlanarSystem(
            [Bar],
            PlanarSystem.StandardGravity,
            [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero), friction: 0.05)],
            loads:
            [
                new ExternalLoad("wind", "bar", new Vector2d(0.3, 0), [new LoadPoint(0.2003, 0, 0), new LoadPoint(0.7001, 4, -1), new LoadPoint(1.3007, -2, 0)]),
            ]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d((-Bar.EndA).Rotated(-1.2), -1.2), default);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 20_000, (_, s) => drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        Assert.True(Math.Abs(system.ExternalWork(state.Values)) > 0.1, $"Only {system.ExternalWork(state.Values)} J of external work.");
        Assert.InRange(drift, 0.0, 1e-10);
    }

    [Fact]
    public void Rk4KeepsFourthOrderAcrossLoadBreakpoints()
    {
        static double FinalAngle(double dt)
        {
            var system = new PlanarSystem(
                [Bar],
                PlanarSystem.StandardGravity,
                [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero))],
                loads: [new ExternalLoad("gust", "bar", Bar.EndB, [new LoadPoint(0.10037, 0, 0), new LoadPoint(0.30071, 60, 30), new LoadPoint(0.70011, 0, 0)])]);
            var state = system.CreateState();
            system.SetBody(state.Values, 0, new Pose2d((-Bar.EndA).Rotated(-1.0), -1.0), default);
            FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), dt, (long)Math.Round(1.0 / dt));
            return system.GetPose(state.Values, 0).Angle;
        }

        // Measured: 2.1e-10, 1.3e-11, 8.1e-13 at 4e-4, 2e-4, 1e-4 s (15.9x, 16.0x),
        // and 15.7-16.3x per halving from 1.6e-3 s down to 5e-5 s.
        var reference = FinalAngle(2.5e-5);
        var errors = new[] { 4e-4, 2e-4, 1e-4 }.Select(dt => Math.Abs(FinalAngle(dt) - reference)).ToArray();
        Assert.InRange(errors[0] / errors[1], 14.0, 18.0);
        Assert.InRange(errors[1] / errors[2], 14.0, 18.0);
    }

    [Fact]
    public void RejectsBadLoads()
    {
        Assert.Throws<ArgumentException>(() => new ExternalLoad("a", "bar", Vector2d.Zero, []));
        Assert.Throws<ArgumentException>(() => new ExternalLoad("a", "bar", Vector2d.Zero, [new LoadPoint(1, 0, 0), new LoadPoint(1, 2, 0)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ExternalLoad("a", "bar", Vector2d.Zero, [new LoadPoint(0, double.NaN, 0)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => ExternalLoad.Pulse("a", "bar", Vector2d.Zero, new Vector2d(1, 0), 0, 0));
        Assert.Throws<ArgumentException>(() => new PlanarSystem([Bar], Vector2d.Zero, loads: [new ExternalLoad("a", "rod", Vector2d.Zero, [new LoadPoint(0, 1, 0)])]));
        ExternalLoad push = new("a", "bar", Vector2d.Zero, [new LoadPoint(0, 1, 0)]);
        Assert.Throws<ArgumentException>(() => new PlanarSystem([Bar], Vector2d.Zero, loads: [push, push]));
    }

    [Fact]
    public void PulseHasTheRequestedImpulse()
    {
        var pulse = ExternalLoad.Pulse("tap", "bar", Vector2d.Zero, new Vector2d(0.3, -0.6), start: 1.0, duration: 0.01);

        Assert.Equal([1.0, 1.005, 1.01], pulse.Points.Select(p => p.Time));
        Assert.Equal(60.0, pulse.Points[1].Fx, 9);
        Assert.Equal(-120.0, pulse.Points[1].Fy, 9);
        Assert.Equal(Vector2d.Zero, pulse.ForceAt(0.5));
        Assert.Equal(new Vector2d(30.0, -60.0), pulse.ForceAt(1.0025));
    }
}
