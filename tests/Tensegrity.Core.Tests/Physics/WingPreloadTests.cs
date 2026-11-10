using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Hyperextension stops on the wing (PLAN.md M6, test 10 compared): the
/// elbow and wrist tendons designed shorter so they preload each joint up
/// into a stop, against the plain tendon lock. Gravity pulls the joints
/// away from these stops, so they barely move the fold: the lock fails at
/// the same tendon rest length either way, to 1e-5, and the preload only buys the
/// margin of its own extra tendon stretch.
/// </summary>
public class WingPreloadTests
{
    private const double Dt = 1e-4;

    /// <summary>Distance either side of a predicted threshold that the simulation is checked at, as a rest-length scale.</summary>
    private const double Bracket = 1e-4;

    private static readonly string[] Joints = [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist];
    private static readonly LockingWingGeometry Geometry = ReferenceTopologies.PreloadedWing;
    private static readonly PlanarModel ScaleOne = Wing.Preloaded();
    private static readonly ChainStatics Statics = ChainStatics.ForWing(ScaleOne, Geometry);
    private static readonly PlanarModel PlainScaleOne = Wing.Locked();
    private static readonly ChainStatics Plain = ChainStatics.ForWing(PlainScaleOne);

    private static double Degrees(double radians) => radians * 180 / Math.PI;

    private static double[] Scales(int tendon, double scale)
    {
        var scales = new[] { 1.0, 1.0, 1.0 };
        scales[tendon] = scale;
        return scales;
    }

    /// <summary>Runs the wing and returns the lowest and highest joint angle reached at each joint, and the final state.</summary>
    private static (double[] MinJoint, double[] MaxJoint, double[] Final) Run(PlanarModel model, SystemState state, double duration)
    {
        var system = model.System;
        var min = new double[3];
        var max = new double[3];
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), Dt, (long)Math.Round(duration / Dt), (_, s) =>
        {
            for (var p = 0; p < 3; p++)
            {
                var angle = system.GetJointAngle(s.Values, p).Angle;
                min[p] = Math.Min(min[p], angle);
                max[p] = Math.Max(max[p], angle);
            }
        });
        return (min, max, state.Values);
    }

    [Theory]
    [InlineData(0.0, 0.0, 0.0)]
    [InlineData(0.0, 0.05, 0.1)]
    [InlineData(-0.1, 0.02, -0.15)]
    [InlineData(0.3, -0.5, -1.1)]
    public void GradientWithStopsIsTheDerivativeOfThePotential(double t1, double t2, double t3)
    {
        // Poses on, past and clear of the stops (at −1.4° and −0.9°).
        double[] angles = [t1, t2, t3];
        double[] scales = [1, 1, 1];
        var gradient = Statics.Gradient(angles, scales);
        for (var k = 0; k < 3; k++)
        {
            var plus = (double[])angles.Clone();
            var minus = (double[])angles.Clone();
            plus[k] += 1e-6;
            minus[k] -= 1e-6;
            var difference = (Statics.Potential(plus, scales) - Statics.Potential(minus, scales)) / 2e-6;
            Assert.Equal(difference, gradient[k], 1e-7);
        }
    }

    [Fact]
    public void ThePreloadedWingIsStraightOnItsStopsAtZeroTorque()
    {
        var model = Wing.Preloaded();
        var system = model.System;
        var hold = system.SolveHoldingTorques(0, model.InitialState, Joints);
        var stops = new double[3];
        system.GetLimitTorques(model.InitialState, stops);
        var tensions = new double[3];
        var plainTensions = new double[3];
        system.GetCableTensions(model.InitialState, tensions);
        PlainScaleOne.System.GetCableTensions(PlainScaleOne.InitialState, plainTensions);

        foreach (var joint in Joints)
        {
            Assert.InRange(Math.Abs(hold[joint]), 0.0, 1e-12);
        }

        // Each stop pushes its outer link down with exactly the preload.
        Assert.Equal(-ReferenceTopologies.WingElbowPreload, stops[1], 12);
        Assert.Equal(-ReferenceTopologies.WingWristPreload, stops[2], 12);
        Assert.Equal(ReferenceTopologies.WingElbowPreload, Statics.StopTorque([0, 0, 0], 1), 12);
        Assert.Equal(ReferenceTopologies.WingWristPreload, Statics.StopTorque([0, 0, 0], 2), 12);
        Assert.True(Statics.IsStable([0, 0, 0], [1, 1, 1]));

        // The cost: a preload near each joint's gravity moment about doubles
        // its tendon's tension. The shoulder's is unchanged, since the
        // stops are internal to the wing.
        Assert.Equal(plainTensions[0], tensions[0], 9);
        Assert.InRange(tensions[1] / plainTensions[1], 1.9, 2.1);
        Assert.InRange(tensions[2] / plainTensions[2], 1.9, 2.1);
    }

    [Fact]
    public void ThePreloadMovesTheDesignNotTheFold()
    {
        // Measured: the preloaded thresholds are shoulder 1.07348, elbow
        // 1.08595, wrist 1.06727, against the plain 1.07332, 1.06808,
        // 1.06331. In absolute rest length the elbow and wrist folds agree
        // with the plain wing's to under 1 µm, 1e-5 of the rest length: the
        // joint droops away from its stop before it folds, so the stop
        // cannot hold it, and the only difference is that the other joints
        // stay on their stops instead of easing 0.1-0.2°. The design rest
        // lengths are shorter by the extra stretch the preload puts in each
        // tendon (1.67% at the elbow, 0.37% at the wrist, whose tendon is
        // short and lightly loaded), and that is all the margin it buys.
        var preloaded = Enumerable.Range(0, 3).Select(Statics.Fold).ToArray();
        var plain = Enumerable.Range(0, 3).Select(Plain.Fold).ToArray();
        var rest = ScaleOne.System.Cables.Select(c => c.RestLength).ToArray();
        var plainRest = PlainScaleOne.System.Cables.Select(c => c.RestLength).ToArray();

        Assert.Equal(1.07348, preloaded[0].Scale, 4);
        Assert.Equal(1.08595, preloaded[1].Scale, 4);
        Assert.Equal(1.06727, preloaded[2].Scale, 4);
        for (var t = 1; t < 3; t++)
        {
            Assert.True(rest[t] < plainRest[t]);
            Assert.True(preloaded[t].Scale > plain[t].Scale);
            Assert.Equal(plain[t].Scale * plainRest[t], preloaded[t].Scale * rest[t], 1e-6);
            Assert.Equal(0.0, Statics.StopTorque(preloaded[t].Angles, t), 12);
        }

        // The shoulder tendon is unchanged; its fold moves 1.5e-4 because
        // the stops keep the outer links from bending up as the wing drops.
        Assert.Equal(plainRest[0], rest[0], 12);
        Assert.InRange(preloaded[0].Scale - plain[0].Scale, 1e-4, 2e-4);
    }

    [Fact]
    public void StopsWithoutPreloadLeaveTheElbowAndWristThresholds()
    {
        var geometry = new LockingWingGeometry(HyperextensionStops: true);
        var statics = ChainStatics.ForWing(LockingWing.Create(geometry), geometry);

        for (var t = 1; t < 3; t++)
        {
            Assert.Equal(Plain.Fold(t).Scale, statics.Fold(t).Scale, 4);
        }
    }

    [Fact]
    public void LengtheningAPreloadedTendonUnloadsItsStopBeforeTheWingMoves()
    {
        // The wrist stop lets go about where the tendon reaches the plain
        // wing's rest length (scale 1.0036). Until then the joint moves only
        // by the stop's own compliance, at most preload / k = 0.86°; past it
        // the hand droops on its tendon alone.
        var release = PlainScaleOne.System.Cables[2].RestLength / ScaleOne.System.Cables[2].RestLength;
        var onStop = Statics.RestPose([1, 1, release - 0.001]);
        var offStop = Statics.RestPose([1, 1, release + 0.01]);

        Assert.InRange(release, 1.003, 1.004);
        Assert.True(Statics.StopTorque(onStop, 2) > 0);
        Assert.InRange(Degrees(onStop[2] - onStop[1]), -0.86, 0.0);
        Assert.Equal(0.0, Statics.StopTorque(offStop, 2));
        Assert.True(Degrees(offStop[2] - offStop[1]) < -0.86);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void EachPreloadedJointHoldsJustShortOfItsThresholdAndCollapsesJustPastIt(int tendon)
    {
        var fold = Statics.Fold(tendon);

        var below = fold.Scale - Bracket;
        var rest = Statics.RestPose(Scales(tendon, below));
        var held = LockingWing.Create(Geometry, restLengthScales: Scales(tendon, below));
        var (heldMin, _, heldFinal) = Run(held, Wing.At(held, rest, handRate: -0.05), 3.0);

        var above = fold.Scale + Bracket;
        var collapsing = LockingWing.Create(Geometry, restLengthScales: Scales(tendon, above));
        var (collapsedMin, _, _) = Run(collapsing, Wing.At(collapsing, fold.Angles), 6.0);

        Assert.True(Degrees(heldMin[tendon]) > -50, $"Just short of the threshold joint {tendon} bent to {Degrees(heldMin[tendon])}°.");
        for (var b = 0; b < 3; b++)
        {
            Assert.InRange(Math.Abs(held.System.GetPose(heldFinal, b).Angle - rest[b]), 0.0, 1e-3);
        }

        Assert.True(Degrees(collapsedMin[tendon]) < -80, $"Just past the threshold joint {tendon} only reached {Degrees(collapsedMin[tendon])}°.");
    }

    [Fact]
    public void TheStopsCatchTheHandFlickingUpAfterTheGust()
    {
        // Without stops the hand flicks up 52.5° on the gust's rebound with
        // its tendon slack (the locking_wing reference run). Measured with
        // them: the wrist and elbow rebound only into their stops, to +0.8°
        // and +1.4° (1.6° and 2.8° of stop compression). The gust is 5 N m
        // at the wrist against a 0.075 N m preload, so the wrist bends down
        // as far as before (−23.2° against −22.8°); the elbow, whose preload
        // is larger, bends only to −5.8° against −10.3°.
        var plain = Wing.Gusted();
        var preloaded = Wing.PreloadedGusted();

        var (plainMin, plainMax, _) = Run(plain, plain.CreateInitialState(), 1.0);
        var (min, max, _) = Run(preloaded, preloaded.CreateInitialState(), 1.0);

        Assert.True(Degrees(plainMax[2]) > 50);
        Assert.InRange(Degrees(max[2]), 0.0, 2.0);
        Assert.InRange(Degrees(max[1]), 0.0, 2.0);
        Assert.InRange(Degrees(min[2]), Degrees(plainMin[2]) - 1, Degrees(plainMin[2]) + 1);
        Assert.InRange(min[1], 0.7 * plainMin[1], 0.0);
    }

    [Fact]
    public void APreloadNeedsStopsAndCannotPull()
    {
        Assert.Throws<ArgumentException>(() => LockingWing.Create(new LockingWingGeometry(WristPreload: 0.1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => LockingWing.Create(new LockingWingGeometry(HyperextensionStops: true, ElbowPreload: -0.1)));
    }
}
