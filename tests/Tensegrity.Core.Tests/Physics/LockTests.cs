using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 10 (PLAN.md section 7). A pose is locked if it is a stable
/// equilibrium at zero actuator torque. The locking arm holds itself
/// straight out with no torque and returns after a disturbance. Lengthening
/// a tendon past the threshold predicted by the arm's statics
/// (<see cref="ArmStatics"/>, written independently of the simulator)
/// makes it collapse, and just short of the threshold it still holds.
/// </summary>
public class LockTests
{
    private const double Dt = 1e-4;

    /// <summary>
    /// Distance either side of a predicted threshold that the simulation is
    /// checked at, as a rest-length scale: ±0.014 mm on the elbow tendon,
    /// 0.12% of the 8.2% lock margin.
    /// </summary>
    private const double Bracket = 1e-4;

    private static readonly PlanarModel ScaleOne = LockingArm.Create();
    private static readonly ArmStatics Statics = ArmStatics.For(ScaleOne);

    private static double Degrees(double radians) => radians * 180 / Math.PI;

    /// <summary>
    /// Runs the arm and returns the most negative angles reached: the upper
    /// link's, the lower link's (both world), and the elbow's (lower relative
    /// to upper), with the final state.
    /// </summary>
    private static (double MinUpper, double MinLower, double MinElbow, double[] Final) Run(PlanarModel model, SystemState state, double duration)
    {
        var system = model.System;
        double minUpper = 0, minLower = 0, minElbow = 0;
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), system), Dt, (long)Math.Round(duration / Dt), (_, s) =>
        {
            var upper = system.GetPose(s.Values, 0).Angle;
            var lower = system.GetPose(s.Values, 1).Angle;
            minUpper = Math.Min(minUpper, upper);
            minLower = Math.Min(minLower, lower);
            minElbow = Math.Min(minElbow, lower - upper);
        });
        return (minUpper, minLower, minElbow, state.Values);
    }

    [Fact]
    public void ThePredictedThresholdsMatchTheDesignStudy()
    {
        // The design study (a separate finite-difference script) found the
        // folds at scales 1.08187 (elbow, lower link 38.0° down) and 1.08497
        // (shoulder, upper link 37.5° down). The scale peaks flatly, so the
        // angle at the peak is known far less sharply than the scale.
        var elbow = Statics.ElbowFold();
        var shoulder = Statics.ShoulderFold();

        Assert.Equal(1.08187, elbow.Scale, 4);
        Assert.InRange(Degrees(elbow.LowerAngle), -38.5, -37.5);
        Assert.Equal(1.08497, shoulder.Scale, 4);
        Assert.InRange(Degrees(shoulder.UpperAngle), -38.0, -37.0);
    }

    [Theory]
    [InlineData(1.00, 1.05)]
    [InlineData(1.05, 1.00)]
    [InlineData(1.03, 1.06)]
    public void SimulatorAgreesThatTheStaticRestPoseNeedsNoTorque(double shoulderScale, double elbowScale)
    {
        var (t1, t2) = Statics.RestPose(shoulderScale, elbowScale);
        var model = LockingArm.Create(shoulderRestLengthScale: shoulderScale, elbowRestLengthScale: elbowScale);
        var state = Arm.At(model, t1, t2);

        var hold = model.System.SolveHoldingTorques(0, state.Values, [LockingArm.Shoulder, LockingArm.Elbow]);

        Assert.True(Statics.IsStable(t1, t2, shoulderScale, elbowScale));
        Assert.True(t2 < -0.01, "The lengthened tendon should let the arm droop.");
        Assert.InRange(Math.Abs(hold[LockingArm.Shoulder]), 0.0, 1e-10);
        Assert.InRange(Math.Abs(hold[LockingArm.Elbow]), 0.0, 1e-10);
    }

    [Fact]
    public void TheStraightArmIsLockedAndReturnsAfterAKick()
    {
        var model = Arm.Locked();

        var hold = model.System.SolveHoldingTorques(0, model.InitialState, [LockingArm.Shoulder, LockingArm.Elbow]);
        var (minUpper, minLower, minElbow, final) = Run(model, Arm.Kicked(model), 3.0);

        Assert.InRange(Math.Abs(hold[LockingArm.Shoulder]), 0.0, 1e-12);
        Assert.InRange(Math.Abs(hold[LockingArm.Elbow]), 0.0, 1e-12);
        Assert.True(Degrees(minLower) < -1, $"The kick only moved the lower link to {Degrees(minLower)}°.");
        Assert.True(Degrees(minElbow) > -10 && Degrees(minUpper) > -10, $"The kick bent the arm to {Degrees(minUpper)}° / {Degrees(minElbow)}°.");
        Assert.InRange(Math.Abs(Degrees(model.System.GetPose(final, 0).Angle)), 0.0, 0.01);
        Assert.InRange(Math.Abs(Degrees(model.System.GetPose(final, 1).Angle)), 0.0, 0.01);
    }

    [Fact]
    public void ElbowHoldsJustShortOfItsThresholdAndCollapsesJustPastIt()
    {
        var fold = Statics.ElbowFold();

        // Just short: start at the predicted stable pose, nudge the lower link down, and it returns there.
        var below = fold.Scale - Bracket;
        var (t1, t2) = Statics.RestPose(1, below);
        var held = LockingArm.Create(elbowRestLengthScale: below);
        var (_, _, heldElbow, heldFinal) = Run(held, Arm.At(held, t1, t2, lowerRate: -0.05), 3.0);

        // Just past: there is no stable pose; start where the last one was and it collapses.
        var above = fold.Scale + Bracket;
        var collapsing = LockingArm.Create(elbowRestLengthScale: above);
        var (_, _, collapsedElbow, _) = Run(collapsing, Arm.At(collapsing, fold.UpperAngle, fold.LowerAngle), 3.0);

        Assert.True(Degrees(heldElbow) > -50, $"Just short of the threshold the elbow bent to {Degrees(heldElbow)}°.");
        Assert.InRange(Math.Abs(held.System.GetPose(heldFinal, 1).Angle - t2), 0.0, 1e-3);
        Assert.True(Degrees(collapsedElbow) < -90, $"Just past the threshold the elbow only reached {Degrees(collapsedElbow)}°.");
    }

    [Fact]
    public void ShoulderHoldsJustShortOfItsThresholdAndCollapsesJustPastIt()
    {
        var fold = Statics.ShoulderFold();

        var below = fold.Scale - Bracket;
        var (t1, t2) = Statics.RestPose(below, 1);
        var held = LockingArm.Create(shoulderRestLengthScale: below);
        var (heldUpper, _, _, heldFinal) = Run(held, Arm.At(held, t1, t2, lowerRate: -0.05), 3.0);

        var above = fold.Scale + Bracket;
        var collapsing = LockingArm.Create(shoulderRestLengthScale: above);
        var (collapsedUpper, _, _, _) = Run(collapsing, Arm.At(collapsing, fold.UpperAngle, fold.LowerAngle), 3.0);

        Assert.True(Degrees(heldUpper) > -50, $"Just short of the threshold the upper link dropped to {Degrees(heldUpper)}°.");
        Assert.InRange(Math.Abs(held.System.GetPose(heldFinal, 0).Angle - t1), 0.0, 1e-3);
        Assert.True(Degrees(collapsedUpper) < -80, $"Just past the threshold the upper link only reached {Degrees(collapsedUpper)}°.");
    }

    [Fact]
    public void TheReferenceCollapseCaseFallsWithinHalfASecond()
    {
        var model = Arm.Collapsing();

        var (_, _, minElbow, _) = Run(model, model.CreateInitialState(), 0.5);

        Assert.True(Arm.CollapseScale > Statics.ElbowFold().Scale);
        Assert.True(Degrees(minElbow) < -90, $"The elbow only reached {Degrees(minElbow)}°.");
    }
}
