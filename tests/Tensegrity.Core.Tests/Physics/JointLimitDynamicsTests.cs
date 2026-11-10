using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Joint limits in motion: a bar pinned to the ground meets a torsional stop
/// on its angle, which acts as a one-sided spring-damper. Checked against
/// closed forms (static balance, undamped rebound), the energy balance, the
/// angular momentum of a free chain, and RK4 order across stop events.
/// </summary>
public class JointLimitDynamicsTests
{
    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);
    private static readonly PlanarLink Upper = PlanarLink.SlenderBar("upper", mass: 1.0, length: 1.0);
    private static readonly PlanarLink Lower = PlanarLink.SlenderBar("lower", mass: 0.6, length: 0.8);

    private static double PivotInertia => Bar.MomentOfInertiaAbout(Bar.EndA);

    /// <summary>The bar pinned at its end A to the ground origin, so its joint angle is θ.</summary>
    private static PlanarSystem PinnedBar(Vector2d gravity, JointLimit limit) => new(
        [Bar],
        gravity,
        [new PinJoint("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero), limit: limit)]);

    private static SystemState PinnedBarState(PlanarSystem system, double theta, double omega)
    {
        var state = system.CreateState();
        var r = (-Bar.EndA).Rotated(theta);
        system.SetBody(state.Values, 0, new Pose2d(r, theta), new Twist2d(Vector2d.Cross(omega, r), omega));
        return state;
    }

    [Fact]
    public void BarSettlesOnAStopWhereTheSpringBalancesGravity()
    {
        const double k = 500.0;
        var system = PinnedBar(PlanarSystem.StandardGravity, new JointLimit(0.0, double.PositiveInfinity, k, damping: 20.0));
        var state = PinnedBarState(system, 0.0, 0.0);

        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), 1e-4, 40_000);

        // k·δ = m·g·d·cos δ with δ = −θ and d = ℓ/2, solved by Newton.
        var load = Bar.Mass * 9.81 * (Bar.Length / 2);
        var delta = 0.0;
        for (var i = 0; i < 50; i++)
        {
            delta -= ((k * delta) - (load * Math.Cos(delta))) / (k + (load * Math.Sin(delta)));
        }

        var (angle, rate) = system.GetJointAngle(state.Values, 0);
        Assert.Equal(-delta, angle, 10);
        Assert.Equal(0.0, rate, 10);
        Assert.Equal((ContactMode.Pressing, ContactMode.Clear), system.GetLimitModes(state.Values, 0));

        var torques = new double[1];
        system.GetLimitTorques(state.Values, torques);
        Assert.Equal(k * delta, torques[0], 7);

        // The stop holds the bar by itself: no muscle needed.
        var holding = system.SolveHoldingTorques(state.Time, state.Values, ["pivot"]);
        Assert.InRange(Math.Abs(holding.Torques[0]), 0.0, 1e-6);
        Assert.Equal(0.5 * k * delta * delta, system.LimitElasticEnergy(state.Values), 12);
    }

    [Fact]
    public void UndampedStopReboundsElasticallyAfterHalfAPeriod()
    {
        const double k = 500.0;
        const double omega0 = 2.0;
        var system = PinnedBar(Vector2d.Zero, new JointLimit(0.0, double.PositiveInfinity, k));
        var state = PinnedBarState(system, 0.5, -omega0);
        var energy = system.TotalEnergy(state.Values);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);

        FixedStepRunner.Run(state, stepper, 1e-4, 10_000);

        // Free flight to the stop at 0.25 s, then half a period of
        // I·δ̈ = −k·δ, then free flight back out at the same rate.
        var leaves = 0.25 + (Math.PI * Math.Sqrt(PivotInertia / k));
        var (angle, rate) = system.GetJointAngle(state.Values, 0);
        Assert.Equal(omega0, rate, 9);
        Assert.Equal(omega0 * (1.0 - leaves), angle, 9);
        Assert.Equal(energy, system.TotalEnergy(state.Values), 9);
        Assert.Equal(2, stepper.EventCount);
    }

    [Fact]
    public void DampedStopEnergyBalanceClosesThroughEveryMode()
    {
        var system = PinnedBar(PlanarSystem.StandardGravity, new JointLimit(0.0, double.PositiveInfinity, 500.0, damping: 5.0));
        var state = PinnedBarState(system, 1.0, 0.0);
        var initial = system.TotalEnergy(state.Values);
        var modes = new HashSet<ContactMode>();
        var drift = 0.0;
        var previous = 0.0;
        var monotone = true;

        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), 1e-4, 30_000, (_, s) =>
        {
            modes.Add(system.GetLimitModes(s.Values, 0).Lower);
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial));
            var dissipated = system.DissipatedEnergy(s.Values);
            monotone &= dissipated >= previous;
            previous = dissipated;
        });

        Assert.Equal([ContactMode.Clear, ContactMode.Pressing, ContactMode.Recoiling], modes.Order());
        Assert.True(monotone, "Dissipated energy must never decrease.");
        Assert.True(system.DissipatedEnergy(state.Values) > 1, $"Only {system.DissipatedEnergy(state.Values)} J dissipated.");
        Assert.InRange(drift, 0.0, 1e-9);
    }

    [Fact]
    public void LimitIsInternalSoAFreeChainKeepsItsMomentumBetweenBothStops()
    {
        var limit = new JointLimit(-0.4, 0.4, stiffness: 50.0);
        var system = new PlanarSystem(
            [Upper, Lower],
            Vector2d.Zero,
            [new PinJoint("elbow", Attachment.OnLink("lower", Lower.EndA), Attachment.OnLink("upper", Upper.EndB), limit: limit)]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(Vector2d.Zero, 0), new Twist2d(Vector2d.Zero, 0));
        var r = -Lower.EndA;
        system.SetBody(state.Values, 1, new Pose2d(Upper.EndB + r, 0), new Twist2d(Vector2d.Cross(3.0, r), 3.0));
        var momentum = system.AngularMomentum(state.Values, Vector2d.Zero);
        var energy = system.TotalEnergy(state.Values);
        bool lower = false, upper = false;
        double minAngle = 0, maxAngle = 0;

        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), 1e-4, 30_000, (_, s) =>
        {
            var modes = system.GetLimitModes(s.Values, 0);
            lower |= modes.Lower == ContactMode.Pressing;
            upper |= modes.Upper == ContactMode.Pressing;
            var angle = system.GetJointAngle(s.Values, 0).Angle;
            minAngle = Math.Min(minAngle, angle);
            maxAngle = Math.Max(maxAngle, angle);
        });

        Assert.True(lower && upper, "The elbow should strike both stops.");
        Assert.InRange(minAngle, -0.6, -0.4);
        Assert.InRange(maxAngle, 0.4, 0.6);
        Assert.Equal(momentum, system.AngularMomentum(state.Values, Vector2d.Zero), 9);
        Assert.Equal(energy, system.TotalEnergy(state.Values), 9);
    }

    [Fact]
    public void Rk4KeepsFourthOrderAcrossStopEvents()
    {
        static double FinalAngle(double dt)
        {
            var system = PinnedBar(PlanarSystem.StandardGravity, new JointLimit(0.0, 1.2, 500.0, damping: 5.0));
            var state = PinnedBarState(system, 1.0, 0.0);
            FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), dt, (long)Math.Round(1.0 / dt));
            return system.GetJointAngle(state.Values, 0).Angle;
        }

        // Measured: errors 2.1e-9, 1.4e-10, 7.7e-12, ratios 15.6 and 17.7,
        // with three stop events in the second.
        var reference = FinalAngle(2.5e-5);
        var errors = new[] { 8e-4, 4e-4, 2e-4 }.Select(dt => Math.Abs(FinalAngle(dt) - reference)).ToArray();

        for (var i = 0; i + 1 < errors.Length; i++)
        {
            var ratio = errors[i] / errors[i + 1];
            Assert.InRange(ratio, 12.0, 20.0);
        }
    }

    [Fact]
    public void UnlatchedSystemWithALimitRefusesToEvaluate()
    {
        var system = PinnedBar(Vector2d.Zero, new JointLimit(0.0, 1.0, 100.0));
        var state = PinnedBarState(system, 0.5, 0.0);
        var derivative = new double[system.StateLength];

        Assert.Throws<InvalidOperationException>(() => system.Evaluate(0, state.Values, derivative));
    }
}
