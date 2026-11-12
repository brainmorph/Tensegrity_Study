using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// The two-link toggle arm (PLAN.md M8): a snap latch at the elbow. An
/// over-center tendon pulls the elbow into a stop at straight or a fold stop
/// at −60°, so both shapes are held with zero torque, and an energy barrier
/// lies between them. <see cref="ChainStatics"/> predicts the poles and the
/// barrier independently of the simulator; the simulator holds both poles
/// and snaps between them when a twitch across the elbow is large enough.
/// </summary>
public class ToggleArmTests
{
    private const double Dt = 1e-4;

    private static readonly string[] Joints = [ToggleArm.Shoulder, ToggleArm.Elbow];
    private static readonly double[] One = [1, 1];
    private static readonly ChainStatics Statics = Toggle.Statics();
    private static readonly double[] Straight = Toggle.StraightPole(Statics);
    private static readonly double[] Folded = Toggle.FoldedPole(Statics);
    private static readonly ChainBarrier Barrier = Statics.Barrier(1, Straight, Folded);

    private static double Degrees(double radians) => radians * 180 / Math.PI;

    private static double ElbowAt(double[] pose) => pose[1] - pose[0];

    /// <summary>Runs for <paramref name="duration"/> and returns the elbow angle at the end, the farthest it got from <paramref name="from"/>, and the actuator work.</summary>
    private static (double Final, double Farthest, double Work) Run(PlanarModel model, SystemState state, double duration, double from)
    {
        var system = model.System;
        var farthest = from;
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), Dt, (long)Math.Round(duration / Dt), (_, s) =>
        {
            var elbow = system.GetJointAngle(s.Values, 1).Angle;
            if (Math.Abs(elbow - from) > Math.Abs(farthest - from))
            {
                farthest = elbow;
            }
        });
        return (system.GetJointAngle(state.Values, 1).Angle, farthest, system.ActuatorWork(state.Values));
    }

    [Theory]
    [InlineData(0.0, 0.0)]
    [InlineData(0.02, 0.03)]
    [InlineData(0.01, -0.3)]
    [InlineData(0.0, -1.1)]
    [InlineData(-0.2, -0.7)]
    public void GradientWithBothStopsAndARaisedInsertionIsTheDerivativeOfThePotential(double t1, double t2)
    {
        // On the straight stop, past it, between the stops, past the fold stop (at −60°), and drooping.
        double[] angles = [t1, t2];
        var gradient = Statics.Gradient(angles, One);
        for (var k = 0; k < 2; k++)
        {
            var plus = (double[])angles.Clone();
            var minus = (double[])angles.Clone();
            plus[k] += 1e-6;
            minus[k] -= 1e-6;
            var difference = (Statics.Potential(plus, One) - Statics.Potential(minus, One)) / 2e-6;
            Assert.Equal(difference, gradient[k], 1e-7);
        }
    }

    [Fact]
    public void BothPolesAreHeldWithZeroTorque()
    {
        var model = Toggle.Latched();
        var system = model.System;
        var stops = new double[2];

        // Straight: exact by construction, the stop pushing the lower link down with the press.
        Assert.All(Straight, angle => Assert.InRange(Math.Abs(angle), 0.0, 1e-12));
        var hold = system.SolveHoldingTorques(0, model.InitialState, Joints);
        Assert.InRange(Math.Abs(hold[ToggleArm.Shoulder]), 0.0, 1e-12);
        Assert.InRange(Math.Abs(hold[ToggleArm.Elbow]), 0.0, 1e-12);
        system.GetLimitTorques(model.InitialState, stops);
        Assert.Equal(-ToggleArm.StraightPress(), stops[1], 12);
        Assert.Equal(ToggleArm.StraightPress(), Statics.StopTorque(Straight, 1), 12);
        Assert.InRange(ToggleArm.StraightPress(), 0.39, 0.40);

        // Folded: the predicted pose, put into the simulator, needs no torque
        // either, and its fold stop carries the predicted torque.
        var folded = Arm.At(model, Folded[0], Folded[1]);
        hold = system.SolveHoldingTorques(0, folded.Values, Joints);
        Assert.InRange(Math.Abs(hold[ToggleArm.Shoulder]), 0.0, 1e-9);
        Assert.InRange(Math.Abs(hold[ToggleArm.Elbow]), 0.0, 1e-9);
        system.GetLimitTorques(folded.Values, stops);
        Assert.Equal(Statics.FoldStopTorque(Folded, 1), stops[1], 9);

        // Measured: the elbow sits 3.03° into its fold stop, pressed with
        // 1.06 N m, and the upper link pitches up 0.72° because folding the
        // elbow lowers the moment about the shoulder.
        Assert.InRange(Degrees(ElbowAt(Folded)), -63.04, -63.02);
        Assert.InRange(Degrees(Folded[0]), 0.71, 0.73);
        Assert.InRange(stops[1], 1.05, 1.07);
    }

    [Fact]
    public void ABarrierSeparatesThePoles()
    {
        // The barrier is a saddle of V: no net force, and not stable.
        Assert.All(Statics.Gradient(Barrier.Angles, One), g => Assert.InRange(Math.Abs(g), 0.0, 1e-6));
        Assert.False(Statics.IsStable(Barrier.Angles, One));
        Assert.True(Statics.IsStable(Folded, One));

        var fromStraight = Barrier.Potential - Statics.Potential(Straight, One);
        var fromFolded = Barrier.Potential - Statics.Potential(Folded, One);

        // Measured: the saddle is at −39.47°, 411 mJ above the straight pole
        // and 216 mJ above the folded one. The quasi-static prototype, with
        // the upper link held fixed and hard stops, gave −39.5°, 415 and
        // 189 mJ. Gravity helps the fold, so the folded pole has the lower barrier.
        Assert.InRange(Degrees(Barrier.JointAngle), -39.48, -39.46);
        Assert.InRange(fromStraight, 0.4110, 0.4114);
        Assert.InRange(fromFolded, 0.2156, 0.2160);
    }

    [Fact]
    public void ThePolesStayPutWhenReleased()
    {
        var model = Toggle.Latched();
        foreach (var pose in new[] { Straight, Folded })
        {
            var (final, farthest, _) = Run(model, Arm.At(model, pose[0], pose[1]), 1.0, ElbowAt(pose));
            Assert.Equal(ElbowAt(pose), final, 1e-6);
            Assert.Equal(ElbowAt(pose), farthest, 1e-6);
        }
    }

    [Theory]
    [InlineData(true, -0.070, false)]
    [InlineData(true, -0.080, true)]
    [InlineData(false, 0.048, false)]
    [InlineData(false, 0.056, true)]
    public void ATwitchUnderTheThresholdHoldsAndOneOverItSnaps(bool fromStraight, double impulse, bool snaps)
    {
        // Measured thresholds for a 20 ms twitch: 0.0746 N m s to fold the
        // straight elbow, 0.0512 N m s to extend the folded one. The
        // impulses here sit 6-9% either side.
        var start = fromStraight ? Straight : Folded;
        var end = snaps ? (fromStraight ? Folded : Straight) : start;
        var model = ToggleArm.Create(actuators: [Toggle.Twitch(impulse, 0.001)]);

        var (final, farthest, _) = Run(model, Arm.At(model, start[0], start[1]), 1.5, ElbowAt(start));

        Assert.Equal(ElbowAt(end), final, 1e-3);
        if (!snaps)
        {
            // Measured: the elbow turns back at −29.7° from straight and at
            // −46.1° from folded, short of the saddle at −39.5° either way.
            Assert.InRange(Degrees(farthest), fromStraight ? -29.9 : -46.3, fromStraight ? -29.5 : -45.9);
        }
    }

    [Theory]
    [InlineData(true, -0.080)]
    [InlineData(false, 0.056)]
    public void TheSnapCostsMoreThanTheBarrier(bool fromStraight, double impulse)
    {
        // Energy is conserved apart from damping, so the twitch must do at
        // least the barrier's worth of work. Measured: these twitches, 7%
        // and 9% over the thresholds, do 3.68x the barrier's work to fold
        // and 2.04x to extend; the rest goes into the tendons' dampers.
        var start = fromStraight ? Straight : Folded;
        var barrier = Barrier.Potential - Statics.Potential(start, One);
        var model = ToggleArm.Create(actuators: [Toggle.Twitch(impulse, 0.001)]);

        var (_, _, work) = Run(model, Arm.At(model, start[0], start[1]), 0.1, ElbowAt(start));

        Assert.InRange(work / barrier, fromStraight ? 3.6 : 2.0, fromStraight ? 3.8 : 2.1);
    }

    [Fact]
    public void AKickToTheLowerLinkIsShieldedByTheShoulder()
    {
        // A kick to the lower link with the barrier's worth of kinetic energy
        // (13 rad/s about the elbow, 0.414 J) mostly swings the whole arm
        // about the softer shoulder: the elbow bends only 4.6° and returns.
        // Snapping the elbow takes a couple across it, like a muscle.
        var model = Toggle.Latched();
        var state = Arm.At(model, 0, 0, -13.0);
        Assert.True(model.System.KineticEnergy(state.Values) > Barrier.Potential - Statics.Potential(Straight, One));

        var (final, farthest, _) = Run(model, state, 1.5, 0);

        Assert.InRange(Degrees(farthest), -4.7, -4.4);
        Assert.Equal(0, final, 1e-3);
    }

    [Fact]
    public void TheSnapScenarioFoldsThenExtends()
    {
        var model = Toggle.Snapping();
        var system = model.System;
        var state = model.CreateInitialState();
        var atHalf = double.NaN;
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), Dt, 10_000, (step, s) =>
        {
            if (step == 5_000)
            {
                atHalf = system.GetJointAngle(s.Values, 1).Angle;
            }
        });

        // Folded at 0.5 s and straight again at 1 s, each still ringing on
        // its lightly damped stop: measured −62.4° (pole −63.0°) and −0.48°.
        var oneDegree = Math.PI / 180;
        Assert.Equal(ElbowAt(Folded), atHalf, oneDegree);
        Assert.Equal(0, system.GetJointAngle(state.Values, 1).Angle, oneDegree);
    }
}
