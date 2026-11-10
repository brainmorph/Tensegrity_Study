using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 11 (PLAN.md section 7): the effort to hold a pose is the joint
/// torque that keeps every acceleration at zero. Checked against
/// closed-form statics, against the dynamics (the torques really hold the
/// pose), and on a structure that holds itself.
/// </summary>
public class HoldingTorqueTests
{
    private const double G = 9.81;

    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);
    private static readonly PlanarLink Upper = PlanarLink.SlenderBar("upper", mass: 1.0, length: 1.0);
    private static readonly PlanarLink Lower = PlanarLink.SlenderBar("lower", mass: 0.6, length: 0.8);

    private static PinJoint Pivot() => new("pivot", Attachment.OnLink("bar", Bar.EndA), Attachment.Ground(Vector2d.Zero));

    private static PinJoint Shoulder() => new("shoulder", Attachment.OnLink("upper", Upper.EndA), Attachment.Ground(Vector2d.Zero));

    private static PinJoint Elbow() => new("elbow", Attachment.OnLink("lower", Lower.EndA), Attachment.OnLink("upper", Upper.EndB));

    /// <summary>A two-link arm at rest with the upper link at θ1 and the lower at θ2, both pins satisfied exactly.</summary>
    private static (PlanarSystem System, SystemState State) Arm(double theta1, double theta2, IEnumerable<JointActuator>? actuators = null)
    {
        var system = new PlanarSystem([Upper, Lower], PlanarSystem.StandardGravity, [Shoulder(), Elbow()], actuators: actuators);
        var state = system.CreateState();
        var upperCenter = (-Upper.EndA).Rotated(theta1);
        var elbow = upperCenter + Upper.EndB.Rotated(theta1);
        system.SetBody(state.Values, 0, new Pose2d(upperCenter, theta1), default);
        system.SetBody(state.Values, 1, new Pose2d(elbow + (-Lower.EndA).Rotated(theta2), theta2), default);
        return (system, state);
    }

    /// <summary>The torque that cancels gravity's moment about <paramref name="about"/> of a mass at <paramref name="center"/>.</summary>
    private static double CancelGravity(double mass, Vector2d center, Vector2d about) =>
        -Vector2d.Cross(center - about, new Vector2d(0, -mass * G));

    [Fact]
    public void HorizontalBarNeedsItsWeightTimesHalfItsLength()
    {
        var system = new PlanarSystem([Bar], PlanarSystem.StandardGravity, [Pivot()]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(Bar.Length / 2, 0), 0), default);

        var hold = system.SolveHoldingTorques(0, state.Values, ["pivot"]);

        Assert.Equal(Bar.Mass * G * Bar.Length / 2, hold["pivot"], 12);
        Assert.InRange(hold.MaxLinearAcceleration, 0.0, 1e-12);
        Assert.InRange(hold.MaxAngularAcceleration, 0.0, 1e-12);
    }

    [Theory]
    [InlineData(0.4, 1.3)]
    [InlineData(-0.9, -2.2)]
    [InlineData(2.0, 0.1)]
    public void ArmTorquesMatchClosedFormStatics(double theta1, double theta2)
    {
        var (system, state) = Arm(theta1, theta2);
        var upper = system.GetPose(state.Values, 0).Position;
        var lower = system.GetPose(state.Values, 1).Position;
        var elbow = system.WorldPoint(state.Values, Elbow().B);

        var hold = system.SolveHoldingTorques(0, state.Values, ["shoulder", "elbow"]);

        // The shoulder carries both links' weight; the elbow only the lower link's.
        var shoulder = CancelGravity(Upper.Mass, upper, Vector2d.Zero) + CancelGravity(Lower.Mass, lower, Vector2d.Zero);
        Assert.Equal(shoulder, hold["shoulder"], 11);
        Assert.Equal(CancelGravity(Lower.Mass, lower, elbow), hold["elbow"], 11);
        Assert.InRange(hold.MaxLinearAcceleration, 0.0, 1e-11);
        Assert.InRange(hold.MaxAngularAcceleration, 0.0, 1e-11);
    }

    [Fact]
    public void TheSolvedTorquesHoldTheArmStillInTheDynamics()
    {
        var (probe, probeState) = Arm(0.4, 1.3);
        var hold = probe.SolveHoldingTorques(0, probeState.Values, ["shoulder", "elbow"]);
        var (system, state) = Arm(0.4, 1.3,
        [
            new JointActuator("shoulder_motor", "shoulder", Schedule.Constant(hold["shoulder"])),
            new JointActuator("elbow_motor", "elbow", Schedule.Constant(hold["elbow"])),
        ]);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-3, 1000);

        Assert.InRange(Math.Abs(system.GetPose(state.Values, 0).Angle - 0.4), 0.0, 1e-10);
        Assert.InRange(Math.Abs(system.GetPose(state.Values, 1).Angle - 1.3), 0.0, 1e-10);

        // Once held, the arm with its motors needs no further torque.
        var extra = system.SolveHoldingTorques(1, state.Values, ["shoulder", "elbow"]);
        Assert.InRange(Math.Abs(extra["shoulder"]), 0.0, 1e-9);
        Assert.InRange(Math.Abs(extra["elbow"]), 0.0, 1e-9);
    }

    [Fact]
    public void AHangingArmNeedsNoTorque()
    {
        var (system, state) = Arm(-Math.PI / 2, -Math.PI / 2);

        var hold = system.SolveHoldingTorques(0, state.Values, ["shoulder", "elbow"]);

        Assert.InRange(Math.Abs(hold["shoulder"]), 0.0, 1e-13);
        Assert.InRange(Math.Abs(hold["elbow"]), 0.0, 1e-13);
    }

    [Fact]
    public void ACableTakesItsShareOfTheLoad()
    {
        // A vertical cable from (ℓ, 1) to the tip of the horizontal bar at
        // (ℓ, 0): length 1 m, L0 0.95 m, k = 19/0.95 = 20 N/m, so T = 1 N
        // upward at moment arm ℓ.
        var cable = new Cable("stay", Attachment.Ground(new Vector2d(Bar.Length, 1.0)), Attachment.OnLink("bar", Bar.EndB), 0.95, 19.0);
        var system = new PlanarSystem([Bar], PlanarSystem.StandardGravity, [Pivot()], [cable]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(Bar.Length / 2, 0), 0), default);

        var hold = system.SolveHoldingTorques(0, state.Values, ["pivot"]);

        Assert.Equal((Bar.Mass * G * Bar.Length / 2) - (1.0 * Bar.Length), hold["pivot"], 12);
    }

    [Fact]
    public void ThePrestressedXHoldsItselfWithZeroEffort()
    {
        var model = PlanarX.Pinned();

        var hold = model.System.SolveHoldingTorques(0, model.InitialState, ["foot1", "foot2"]);

        Assert.InRange(Math.Abs(hold["foot1"]), 0.0, 1e-9);
        Assert.InRange(Math.Abs(hold["foot2"]), 0.0, 1e-9);
        Assert.InRange(hold.MaxLinearAcceleration, 0.0, 1e-9);
    }

    [Fact]
    public void AnUnderActuatedArmReportsTheAccelerationItCannotCancel()
    {
        var (bent, bentState) = Arm(0.4, 1.3);
        var (hanging, hangingState) = Arm(-Math.PI / 2, -Math.PI / 2);

        var free = bent.SolveHoldingTorques(0, bentState.Values, ["elbow"]);
        var fine = hanging.SolveHoldingTorques(0, hangingState.Values, ["elbow"]);

        Assert.True(free.MaxAngularAcceleration > 1, $"Expected the free shoulder to swing, got {free.MaxAngularAcceleration} rad/s².");
        Assert.InRange(fine.MaxAngularAcceleration, 0.0, 1e-12);
    }

    [Fact]
    public void ARigidStructureHasNoHoldingTorque()
    {
        // Ground–bar–bar–ground with three non-collinear pins: a rigid triangle.
        var a = PlanarLink.SlenderBar("a", 1.0, 1.0);
        var b = PlanarLink.SlenderBar("b", 1.0, 1.0);
        var system = new PlanarSystem(
            [a, b],
            PlanarSystem.StandardGravity,
            [
                new PinJoint("root", Attachment.OnLink("a", a.EndA), Attachment.Ground(Vector2d.Zero)),
                new PinJoint("knee", Attachment.OnLink("b", b.EndA), Attachment.OnLink("a", a.EndB)),
                new PinJoint("tip", Attachment.OnLink("b", b.EndB), Attachment.Ground(new Vector2d(1, 1))),
            ]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0.5, 0), 0), default);
        system.SetBody(state.Values, 1, new Pose2d(new Vector2d(1, 0.5), Math.PI / 2), default);

        var error = Assert.Throws<InvalidOperationException>(() => system.SolveHoldingTorques(0, state.Values, ["knee"]));
        Assert.Contains("cannot move anything", error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void RejectsUnknownAndRepeatedPins()
    {
        var (system, state) = Arm(0.4, 1.3);

        Assert.Throws<ArgumentException>(() => system.SolveHoldingTorques(0, state.Values, ["wrist"]));
        Assert.Throws<ArgumentException>(() => system.SolveHoldingTorques(0, state.Values, ["elbow", "elbow"]));
        Assert.Throws<KeyNotFoundException>(() => system.SolveHoldingTorques(0, state.Values, ["elbow"])["shoulder"]);
    }
}
