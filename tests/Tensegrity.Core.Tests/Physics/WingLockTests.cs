using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 10 (PLAN.md section 7) for the three-link wing: the straight pose
/// is locked at zero torque, and each tendon has a threshold, predicted by
/// <see cref="ChainStatics"/> independently of the simulator, past which
/// the wing collapses.
/// </summary>
public class WingLockTests
{
    private const double Dt = 1e-4;

    /// <summary>Distance either side of a predicted threshold that the simulation is checked at, as a rest-length scale.</summary>
    private const double Bracket = 1e-4;

    private static readonly string[] Joints = [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist];
    private static readonly PlanarModel ScaleOne = LockingWing.Create();
    private static readonly ChainStatics Statics = ChainStatics.ForWing(ScaleOne);

    private static double Degrees(double radians) => radians * 180 / Math.PI;

    private static double[] Scales(int tendon, double scale)
    {
        var scales = new[] { 1.0, 1.0, 1.0 };
        scales[tendon] = scale;
        return scales;
    }

    /// <summary>Runs the wing and returns the most negative joint angle reached at each joint, and the final state.</summary>
    private static (double[] MinJoint, double[] Final) Run(PlanarModel model, SystemState state, double duration)
    {
        var system = model.System;
        var min = new double[3];
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), Dt, (long)Math.Round(duration / Dt), (_, s) =>
        {
            for (var p = 0; p < 3; p++)
            {
                min[p] = Math.Min(min[p], system.GetJointAngle(s.Values, p).Angle);
            }
        });
        return (min, state.Values);
    }

    [Theory]
    [InlineData(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)]
    [InlineData(-0.1, -0.2, -0.3, 1.0, 1.02, 1.0)]
    [InlineData(0.3, -0.5, -1.1, 1.05, 1.0, 1.08)]
    [InlineData(-0.4, 0.2, 0.6, 1.0, 1.3, 1.0)]
    public void ChainGradientIsTheDerivativeOfItsPotential(double t1, double t2, double t3, double s1, double s2, double s3)
    {
        // Guards the chain rule in world angles: turning one link alone
        // carries the outer links along without turning them.
        double[] angles = [t1, t2, t3];
        double[] scales = [s1, s2, s3];
        var gradient = Statics.Gradient(angles, scales);
        for (var k = 0; k < 3; k++)
        {
            var plus = (double[])angles.Clone();
            var minus = (double[])angles.Clone();
            plus[k] += 1e-6;
            minus[k] -= 1e-6;
            var difference = (Statics.Potential(plus, scales) - Statics.Potential(minus, scales)) / 2e-6;
            Assert.Equal(difference, gradient[k], 7);
        }
    }

    [Fact]
    public void ChainStaticsReproducesTheTwoLinkArmThresholds()
    {
        // Two independent statics codes, the two-link ArmStatics and the
        // general chain, must agree on the arm.
        var arm = LockingArm.Create();
        var two = ArmStatics.For(arm);
        var chain = ChainStatics.ForArm(arm);

        Assert.Equal(two.ShoulderFold().Scale, chain.Fold(0).Scale, 12);
        Assert.Equal(two.ElbowFold().Scale, chain.Fold(1).Scale, 12);
    }

    [Fact]
    public void ThePredictedThresholdsShrinkOutwardAndBelowTheArms()
    {
        // Measured: shoulder 1.07332, elbow 1.06808, wrist 1.06331, each
        // with its own link and those outboard of it about 33° down. All
        // three are below the two-link arm's 1.082-1.085: the extra link
        // makes the lock less tolerant.
        var folds = Enumerable.Range(0, 3).Select(Statics.Fold).ToArray();

        Assert.Equal(1.07332, folds[0].Scale, 4);
        Assert.Equal(1.06808, folds[1].Scale, 4);
        Assert.Equal(1.06331, folds[2].Scale, 4);
        Assert.True(folds[0].Scale > folds[1].Scale && folds[1].Scale > folds[2].Scale);
        Assert.InRange(Degrees(folds[2].Angles[2]), -34.0, -32.0);
        Assert.InRange(Degrees(folds[2].Angles[1]), -1.0, 1.0);
    }

    [Fact]
    public void TheStraightWingIsLockedAndReturnsAfterAKickToTheHand()
    {
        var model = Wing.Locked();
        var hold = model.System.SolveHoldingTorques(0, model.InitialState, Joints);
        // The slowest mode, the whole wing pitching on the shoulder tendon,
        // has a period near 2 s and loses about 3x a second: 0.14° is left
        // at 3 s and 0.01° at 6 s.
        var (minJoint, final) = Run(model, Wing.At(model, [0, 0, 0], handRate: -10.0), 6.0);

        Assert.True(Statics.IsStable([0, 0, 0], [1, 1, 1]));
        foreach (var joint in Joints)
        {
            Assert.InRange(Math.Abs(hold[joint]), 0.0, 1e-12);
        }

        // Measured: the joints bend to −3.8° (shoulder), −2.6° (elbow) and −3.2° (wrist).
        Assert.True(Degrees(minJoint[2]) < -2, $"The kick only moved the wrist to {Degrees(minJoint[2])}°.");
        Assert.True(minJoint.All(a => Degrees(a) > -10), $"The kick bent the wing to {string.Join(", ", minJoint.Select(Degrees))}°.");
        for (var b = 0; b < 3; b++)
        {
            Assert.InRange(Math.Abs(Degrees(model.System.GetPose(final, b).Angle)), 0.0, 0.02);
        }
    }

    [Theory]
    [InlineData(1.00, 1.00, 1.04)]
    [InlineData(1.00, 1.05, 1.00)]
    [InlineData(1.03, 1.03, 1.03)]
    public void SimulatorAgreesThatTheStaticRestPoseNeedsNoTorque(double shoulder, double elbow, double wrist)
    {
        var angles = Statics.RestPose([shoulder, elbow, wrist]);
        var model = LockingWing.Create(restLengthScales: [shoulder, elbow, wrist]);

        var hold = model.System.SolveHoldingTorques(0, Wing.At(model, angles).Values, Joints);

        Assert.True(angles[2] < -0.01, "Lengthened tendons should let the wing droop.");
        foreach (var joint in Joints)
        {
            Assert.InRange(Math.Abs(hold[joint]), 0.0, 1e-10);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void EachJointHoldsJustShortOfItsThresholdAndCollapsesJustPastIt(int tendon)
    {
        var fold = Statics.Fold(tendon);

        // Just short: start at the predicted stable pose, nudge the hand down, and it returns there.
        var below = fold.Scale - Bracket;
        var rest = Statics.RestPose(Scales(tendon, below));
        var held = LockingWing.Create(restLengthScales: Scales(tendon, below));
        var (heldMin, heldFinal) = Run(held, Wing.At(held, rest, handRate: -0.05), 3.0);

        // Just past: no stable pose is left; start where the last one was and
        // it collapses. Just past a fold the drift starts slowly (the
        // shoulder creeps 4° in 2.5 s before it goes, at about 3.3 s), so run 6 s.
        var above = fold.Scale + Bracket;
        var collapsing = LockingWing.Create(restLengthScales: Scales(tendon, above));
        var (collapsedMin, _) = Run(collapsing, Wing.At(collapsing, fold.Angles), 6.0);

        Assert.True(Degrees(heldMin[tendon]) > -50, $"Just short of the threshold joint {tendon} bent to {Degrees(heldMin[tendon])}°.");
        for (var b = 0; b < 3; b++)
        {
            Assert.InRange(Math.Abs(held.System.GetPose(heldFinal, b).Angle - rest[b]), 0.0, 1e-3);
        }

        Assert.True(Degrees(collapsedMin[tendon]) < -80, $"Just past the threshold joint {tendon} only reached {Degrees(collapsedMin[tendon])}°.");
    }

    [Fact]
    public void TheWristStopCatchesAHandThrownDownPastItsLock()
    {
        // Released at rest, a collapsing hand only hangs, folding to about
        // −147° and missing the stop. Thrown down at 20 rad/s it would pass
        // through the lower link (measured −186°); the stop catches it a few
        // degrees past 150°.
        var free = LockingWing.Create(new LockingWingGeometry(WristFoldStop: null), restLengthScales: [1, 1, 1.10]);
        var stopped = LockingWing.Create(restLengthScales: [1, 1, 1.10]);

        var (freeMin, _) = Run(free, Wing.At(free, [0, 0, 0], handRate: -20.0), 1.5);
        var (stoppedMin, _) = Run(stopped, Wing.At(stopped, [0, 0, 0], handRate: -20.0), 1.5);

        Assert.True(Degrees(freeMin[2]) < -180, $"Without the stop the hand only folded to {Degrees(freeMin[2])}°.");
        Assert.InRange(Degrees(stoppedMin[2]), -160.0, -150.0);
    }
}
