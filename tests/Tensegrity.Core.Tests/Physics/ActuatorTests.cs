using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Joint actuators (PLAN.md 4.1): a torque couple across a pin, following a
/// time schedule, whose work enters the energy balance (PLAN.md 4.3).
/// </summary>
public class ActuatorTests
{
    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);
    private static readonly PlanarLink Upper = PlanarLink.SlenderBar("upper", mass: 1.0, length: 1.0);
    private static readonly PlanarLink Lower = PlanarLink.SlenderBar("lower", mass: 0.6, length: 0.8);

    /// <summary>The bar pinned to the ground by its end A at the origin, driven at the pivot.</summary>
    private static PlanarSystem PinnedBar(Vector2d gravity, Schedule schedule, IEnumerable<Cable>? cables = null) => new(
        [Bar],
        gravity,
        [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero))],
        cables,
        actuators: [new JointActuator("motor", "pivot", schedule)]);

    /// <summary>The pinned bar at angle θ and rate ω, with the pin satisfied exactly.</summary>
    private static SystemState PinnedBarState(PlanarSystem system, double theta, double omega)
    {
        var state = system.CreateState();
        var r = (-Bar.EndA).Rotated(theta);
        system.SetBody(state.Values, 0, new Pose2d(r, theta), new Twist2d(Vector2d.Cross(omega, r), omega));
        return state;
    }

    private static double PivotInertia => Bar.MomentOfInertiaAbout(Bar.EndA);

    /// <summary>
    /// Exact angle at time <paramref name="t"/> of a body with inertia
    /// <paramref name="inertia"/> driven from rest by <paramref name="schedule"/>
    /// alone: θ̈ = τ(t)/I, integrated segment by segment in closed form.
    /// </summary>
    private static double DrivenAngle(Schedule schedule, double inertia, double t)
    {
        double theta = 0, omega = 0, now = 0;
        var cuts = schedule.Points.Select(p => p.Time).Where(c => c > 0 && c < t).Append(t);
        foreach (var cut in cuts)
        {
            var h = cut - now;
            var torque = schedule.ValueAt(now);
            var slope = (schedule.ValueAt(cut) - torque) / h;
            theta += (omega * h) + (((torque * h * h / 2) + (slope * h * h * h / 6)) / inertia);
            omega += ((torque * h) + (slope * h * h / 2)) / inertia;
            now = cut;
        }

        return theta;
    }

    [Fact]
    public void ConstantTorqueSpinsAPinnedBarAtTauOverPivotInertia()
    {
        const double tau = 0.7;
        var system = PinnedBar(Vector2d.Zero, Schedule.Constant(tau));
        var state = PinnedBarState(system, 0.3, 0.0);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 1000);

        var alpha = tau / PivotInertia;
        Assert.Equal(0.3 + (0.5 * alpha), system.GetPose(state.Values, 0).Angle, 10);
        Assert.Equal(alpha, system.GetTwist(state.Values, 0).Angular, 10);

        // All the work went into kinetic energy.
        var expectedWork = tau * 0.5 * alpha;
        Assert.Equal(expectedWork, system.ActuatorWork(state.Values), 10);
        Assert.Equal(expectedWork, system.KineticEnergy(state.Values), 10);
    }

    [Fact]
    public void TorqueEqualToTheGravityMomentHoldsAHorizontalBarStill()
    {
        // At θ = 0 the center is at (ℓ/2, 0): gravity's moment about the pivot is −m·g·ℓ/2.
        var hold = Bar.Mass * 9.81 * Bar.Length / 2;
        var system = PinnedBar(PlanarSystem.StandardGravity, Schedule.Constant(hold));
        var state = PinnedBarState(system, 0.0, 0.0);
        var dy = new double[system.StateLength];

        system.Evaluate(0, state.Values, dy);
        Assert.InRange(Math.Abs(dy[system.VelocityIndex(0) + 2]), 0.0, 1e-12);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 2000);
        Assert.InRange(Math.Abs(system.GetPose(state.Values, 0).Angle), 0.0, 1e-12);
        Assert.InRange(Math.Abs(system.ActuatorWork(state.Values)), 0.0, 1e-15);
    }

    [Fact]
    public void ElbowTorqueIsInternalSoAFreeChainKeepsItsMomentum()
    {
        var system = new PlanarSystem(
            [Upper, Lower],
            Vector2d.Zero,
            [new PinJoint("elbow", Attachment.OnLink("lower", Lower.EndA), Attachment.OnLink("upper", Upper.EndB))],
            actuators: [new JointActuator("motor", "elbow", Schedule.Constant(0.4))]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(Vector2d.Zero, 0), default);
        system.SetBody(state.Values, 1, new Pose2d(Upper.EndB - Lower.EndA, 0), default);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 1000);

        var lower = system.GetTwist(state.Values, 1).Angular;
        var upper = system.GetTwist(state.Values, 0).Angular;
        var momentum = (Upper.Mass * system.GetTwist(state.Values, 0).Linear) + (Lower.Mass * system.GetTwist(state.Values, 1).Linear);

        // Positive torque turns end A (lower) counterclockwise relative to end B (upper).
        Assert.True(lower > 0 && upper < 0, $"Expected lower to turn ccw and upper cw, got {lower} and {upper}.");
        // RK4 does not conserve angular momentum exactly: measured 1.2e-10
        // here, falling 16x per dt halving, against link momenta near 0.1.
        Assert.InRange(Math.Abs(system.AngularMomentum(state.Values, Vector2d.Zero)), 0.0, 1e-9);
        Assert.InRange(momentum.Length, 0.0, 1e-13);
        Assert.True(system.ActuatorWork(state.Values) > 0.01);

        // Kinetic energy equals the work put in: measured 4.5e-10 off, also
        // 16x down per dt halving.
        Assert.InRange(Math.Abs(system.TotalEnergy(state.Values)), 0.0, 1e-8);
    }

    [Fact]
    public void RampedTorqueMatchesTheClosedFormWithEveryBreakpointSplit()
    {
        // Breakpoints off the step grid, so each one falls inside a step.
        var schedule = new Schedule([new SchedulePoint(0.10037, 0), new SchedulePoint(0.30071, 0.9), new SchedulePoint(0.70011, -0.3)]);
        var system = PinnedBar(Vector2d.Zero, schedule);
        var state = PinnedBarState(system, 0.0, 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);

        FixedStepRunner.Run(state, stepper, 2e-3, 500);

        // What remains is RK4's error on the pinned bar's circular motion in
        // maximal coordinates: measured 1.8e-12, 16x down per dt halving.
        Assert.Equal(3, stepper.EventCount);
        Assert.Equal(DrivenAngle(schedule, PivotInertia, 1.0), system.GetPose(state.Values, 0).Angle, 11);
    }

    [Fact]
    public void RampedTorqueUnderGravityConvergesAtFourthOrder()
    {
        var schedule = new Schedule([new SchedulePoint(0.1003, 0), new SchedulePoint(0.4007, 15), new SchedulePoint(0.9001, 2)]);

        double FinalAngle(double dt)
        {
            var system = PinnedBar(PlanarSystem.StandardGravity, schedule);
            var state = PinnedBarState(system, -Math.PI / 2, 0.0);
            FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), dt, (long)Math.Round(1.5 / dt));
            return system.GetPose(state.Values, 0).Angle;
        }

        var reference = FinalAngle(1.25e-4);
        double[] error = [FinalAngle(4e-3) - reference, FinalAngle(2e-3) - reference, FinalAngle(1e-3) - reference];

        for (var i = 0; i + 1 < error.Length; i++)
        {
            Assert.InRange(error[i] / error[i + 1], 13.0, 19.0);
        }
    }

    [Fact]
    public void EnergyBalanceClosesWithActuatorWorkAndCableDamping()
    {
        // The bar hangs from the pivot. A torque swings it up to the right,
        // where a damped cable from an anchor on the left catches its tip
        // (slack while it hangs, taut near horizontal), then lets it fall.
        var cable = new Cable("stay", Attachment.Ground(new Vector2d(-1.0, 0)), Attachment.OnLink("bar", Bar.EndB), 1.9, 4000, 15);
        var schedule = new Schedule([new SchedulePoint(0.05, 0), new SchedulePoint(0.2501, 40), new SchedulePoint(0.6003, 0)]);
        var system = PinnedBar(PlanarSystem.StandardGravity, schedule, [cable]);
        var state = PinnedBarState(system, -Math.PI / 2, 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000, (_, s) =>
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        Assert.True(stepper.EventCount > 3, "The cable should switch modes as well as the schedule.");
        Assert.True(system.ActuatorWork(state.Values) > 5, $"Only {system.ActuatorWork(state.Values)} J of actuator work.");
        Assert.True(system.DissipatedEnergy(state.Values) > 2, $"Only {system.DissipatedEnergy(state.Values)} J dissipated.");

        // Measured ≈ 7e-12 J with 15 J of work put in and 9.9 J dissipated.
        Assert.InRange(drift, 0.0, 1e-10);
    }

    [Fact]
    public void ActuatorTorquesFollowTheScheduleAtTheGivenTime()
    {
        var schedule = new Schedule([new SchedulePoint(0, 0), new SchedulePoint(2, 4)]);
        var system = PinnedBar(Vector2d.Zero, schedule);
        var torques = new double[1];

        system.GetActuatorTorques(0.5, torques);

        Assert.Equal(1.0, torques[0]);
        Assert.Throws<ArgumentException>(() => system.GetActuatorTorques(0.5, new double[2]));
    }

    [Fact]
    public void ScheduledActuatorsMustBeLatchedButConstantOnesNeedNot()
    {
        var ramp = PinnedBar(Vector2d.Zero, new Schedule([new SchedulePoint(0, 0), new SchedulePoint(1, 1)]));
        var constant = PinnedBar(Vector2d.Zero, Schedule.Constant(1));

        Assert.Throws<InvalidOperationException>(
            () => ramp.Evaluate(0, PinnedBarState(ramp, 0, 0).Values, new double[ramp.StateLength]));
        constant.Evaluate(0, PinnedBarState(constant, 0, 0).Values, new double[constant.StateLength]);
    }

    [Fact]
    public void RejectsBadActuators()
    {
        var pin = new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero));
        var one = Schedule.Constant(1);

        Assert.Throws<ArgumentException>(
            () => new PlanarSystem([Bar], Vector2d.Zero, [pin], actuators: [new JointActuator("m", "missing", one)]));
        Assert.Throws<ArgumentException>(
            () => new PlanarSystem([Bar], Vector2d.Zero, [pin], actuators: [new JointActuator("m", "pivot", one), new JointActuator("m", "pivot", one)]));
        Assert.Throws<ArgumentException>(
            () => new PlanarSystem([Bar], Vector2d.Zero, [pin], actuators: [new JointActuator("m", "pivot", one), new JointActuator("n", "pivot", one)]));
    }
}
