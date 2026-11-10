using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// A moving ground line (PLAN.md M6, the ground drop): its height follows a
/// schedule, it does work N·ḣ on what it carries, and the energy balance
/// counts that work as external work.
/// </summary>
public class MovingGroundTests
{
    private const double Kn = 1e4;
    private const double G = 9.81;

    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 1.0, length: 1.0);

    private static PlanarSystem BarOn(Schedule height) => new(
        [Bar],
        PlanarSystem.StandardGravity,
        ground: new GroundLine(
            [new ContactPoint("left", "bar", Bar.EndA), new ContactPoint("right", "bar", Bar.EndB)],
            Kn,
            60.0,
            0.5,
            1e4,
            60.0,
            heightSchedule: height));

    /// <summary>Static sink of each end below the line, m·g/(2k).</summary>
    private static double Sink => Bar.Mass * G / (2 * Kn);

    /// <summary>The bar lying flat on the line at its scheduled height at t = 0, at rest.</summary>
    private static SystemState Resting(PlanarSystem system)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, system.GetGroundHeight(state.Values) - Sink), 0), default);
        return state;
    }

    private static EventSplittingStepper Stepper(PlanarSystem system) => new(new Rk4Integrator(), system);

    [Fact]
    public void SlowLiftCarriesTheBarUpAndDoesItsWeightTimesTheRise()
    {
        const double rise = 0.1;
        var system = BarOn(Schedule.Ramp(0.2, 0.0, 1.2, rise));
        var state = Resting(system);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 20_000, (_, s) => drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        // Exact up to the breakpoint's placement: 1e-12 s event tolerance times 0.1 m/s.
        Assert.Equal(rise, system.GetGroundHeight(state.Values), 12);
        Assert.Equal(rise - Sink, system.GetPose(state.Values, 0).Position.Y, 9);

        // The lift's speed steps from 0 to 0.1 m/s and back. Each step,
        // taken up through the damped contact springs, dissipates ½·m·Δv²
        // once it rings down: m·v² = 0.01 J for the two. All the rest of the
        // work went into lifting the bar's weight.
        const double speed = rise / 1.0;
        var work = system.ExternalWork(state.Values);
        Assert.Equal(Bar.Mass * speed * speed, system.DissipatedEnergy(state.Values), 8);
        Assert.Equal(Bar.Mass * G * rise, work - system.DissipatedEnergy(state.Values), 9);
        Assert.InRange(drift, 0.0, 1e-10);
    }

    [Fact]
    public void GroundDroppedFasterThanGravityLeavesTheBarFallingFreely()
    {
        // The line falls 0.2 m in 10 ms (20 m/s): the bar cannot follow and is in free fall
        // until it lands on the line's new height.
        const double drop = 0.2;
        var system = BarOn(Schedule.Ramp(0.1, 0.0, 0.11, -drop));
        var state = Resting(system);
        var y0 = system.GetPose(state.Values, 0).Position.Y;
        var stepper = Stepper(system);

        // The ends spring free within a couple of ms, so by 0.115 s the bar has been airborne a while.
        FixedStepRunner.Run(state, stepper, 1e-4, 1_150);
        Assert.Equal(-drop, system.GetGroundHeight(state.Values), 11);
        Assert.Equal((ContactMode.Clear, FrictionMode.Free), system.GetContactModes(state.Time, state.Values, 0));
        var vy1 = system.GetTwist(state.Values, 0).Linear.Y;
        FixedStepRunner.Run(state, stepper, 1e-4, 500);
        Assert.Equal(-G * 0.05, system.GetTwist(state.Values, 0).Linear.Y - vy1, 11);
        Assert.True(system.GetPose(state.Values, 0).Position.Y < y0 - 0.015, "The bar should have fallen ½·g·(65 ms)² ≈ 2 cm.");

        // It lands on the new line and settles there.
        FixedStepRunner.Run(state, stepper, 1e-4, 20_000);
        Assert.Equal(-drop - Sink, system.GetPose(state.Values, 0).Position.Y, 9);
        Assert.Equal((ContactMode.Pressing, FrictionMode.Stick), system.GetContactModes(state.Time, state.Values, 1));
        Assert.Equal(0, stepper.UnresolvedSteps);
    }

    [Fact]
    public void EnergyBalanceClosesThroughADropAndLanding()
    {
        var system = BarOn(Schedule.Ramp(0.1, 0.0, 0.13, -0.15));
        var state = Resting(system);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 15_000, (_, s) => drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        // At 5 m/s the contact dampers' −c·ḣ outweighs the springs at once:
        // the contacts recoil, so the leaving line pushes with no force and
        // does no work. The springs' stored energy goes to dissipation.
        Assert.Equal(0.0, system.ExternalWork(state.Values));
        Assert.InRange(drift, 0.0, 1e-8);
    }

    [Fact]
    public void Rk4KeepsFourthOrderThroughAMovingGround()
    {
        static double FinalY(double dt)
        {
            var system = BarOn(new Schedule([new SchedulePoint(0.05003, 0), new SchedulePoint(0.25007, 0.04), new SchedulePoint(0.30011, -0.02)]));
            var state = Resting(system);
            FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), dt, (long)Math.Round(0.4 / dt));
            return system.GetPose(state.Values, 0).Position.Y;
        }

        var reference = FinalY(2.5e-5);
        var errors = new[] { 4e-4, 2e-4, 1e-4 }.Select(dt => Math.Abs(FinalY(dt) - reference)).ToArray();
        Assert.InRange(errors[0] / errors[1], 12.0, 20.0);
        Assert.InRange(errors[1] / errors[2], 12.0, 20.0);
    }

    [Fact]
    public void FixedHeightAndScheduleCannotBothBeGiven()
    {
        Assert.Throws<ArgumentException>(() => new GroundLine(
            [new ContactPoint("a", "bar", Vector2d.Zero)], Kn, 0, 0.5, 1e4, 60, height: 0.1, heightSchedule: Schedule.Constant(0.1)));
        var moving = new GroundLine([new ContactPoint("a", "bar", Vector2d.Zero)], Kn, 0, 0.5, 1e4, 60, heightSchedule: Schedule.Ramp(1, 0.3, 2, 0));
        Assert.True(moving.IsMoving);
        Assert.Equal(0.3, moving.Height);
    }
}
