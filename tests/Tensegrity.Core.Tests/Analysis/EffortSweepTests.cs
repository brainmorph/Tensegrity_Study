using Tensegrity.Core.Analysis;
using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>
/// Effort curves: holding torque in a fixed pose as one cable's rest length
/// is scaled, and in a pose bent at one joint.
/// </summary>
public class EffortSweepTests
{
    private static readonly string[] Pins = [LockingArm.Shoulder, LockingArm.Elbow];

    private static TopologyDocument KickedArm() => TopologyFiles.ToDocument(ReferenceTopologies.LockingArmKicked());

    [Fact]
    public void ElbowEffortIsItsWeightMomentLessWhatTheTendonCarries()
    {
        double[] scales = [0.98, 0.99, 1.0, 1.003, 1.006, 1.02];

        var points = EffortSweep.Run(KickedArm(), LockingArm.ElbowTendon, Pins, scales);

        // Slack at 1.02: the elbow carries the lower link's whole weight moment,
        // and with tension T the tendon takes the share T / T(1) of it.
        var full = points[^1].Torques[1];
        var design = points[2].Tension;
        Assert.Equal(0.0, points[^1].Tension);
        Assert.Equal(0.12 * 9.81 * 0.35 / 2, full, 12);
        foreach (var point in points)
        {
            Assert.Equal(full * (1 - (point.Tension / design)), point.Torques[1], 10);
            Assert.InRange(Math.Abs(point.Torques[0]), 0.0, 1e-10); // The elbow tendon is internal to the arm.
            Assert.InRange(point.MaxAngularAcceleration, 0.0, 1e-10);
        }

        Assert.InRange(Math.Abs(points[2].Torques[1]), 0.0, 1e-12);
        Assert.True(points[0].Torques[1] < 0, "A short tendon over-lifts: the joint must pull down.");
    }

    [Fact]
    public void TheSweepIgnoresInitialVelocitiesAndLeavesTheDocumentAlone()
    {
        var kicked = KickedArm();
        var before = RunFiles.SerializeTopology(kicked);
        var still = TopologyFiles.ToDocument(LockingArm.Create());

        var a = EffortSweep.Run(kicked, LockingArm.ElbowTendon, Pins, [1.004]);
        var b = EffortSweep.Run(still, LockingArm.ElbowTendon, Pins, [1.004]);

        Assert.Equal(b[0].Torques, a[0].Torques);
        Assert.Equal(b[0].Tension, a[0].Tension);
        Assert.Equal(before, RunFiles.SerializeTopology(kicked));
    }

    [Fact]
    public void UnknownCableIsAnError() =>
        Assert.Throws<ArgumentException>(() => EffortSweep.Run(KickedArm(), "wing_tendon", Pins, [1.0]));

    private static readonly string[] WingJoints = [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist];

    [Theory]
    [InlineData(LockingWing.Shoulder, 1, 1, 1)]
    [InlineData(LockingWing.Elbow, 0, 1, 1)]
    [InlineData(LockingWing.Wrist, 0, 0, 1)]
    public void BendingTurnsTheOuterLinksAndMatchesTheBentPose(string joint, double upper, double forearm, double hand)
    {
        var model = Wing.Locked();
        double[] offsets = [-0.2, -0.05, 0, 0.05, 0.1];

        var points = EffortSweep.Bend(TopologyFiles.ToDocument(model), joint, WingJoints, offsets);

        for (var i = 0; i < offsets.Length; i++)
        {
            var d = offsets[i];
            var bent = Wing.At(model, [upper * d, forearm * d, hand * d]).Values;
            var expected = model.System.SolveHoldingTorques(0, bent, WingJoints);
            var tensions = new double[model.System.Cables.Count];
            model.System.GetCableTensions(bent, tensions);

            Assert.Equal(d, points[i].Offset);
            Assert.Equal(d, points[i].JointAngle, 14); // The straight wing has every joint at zero.
            for (var j = 0; j < WingJoints.Length; j++)
            {
                Assert.Equal(expected.Torques[j], points[i].Torques[j], 11);
                Assert.Equal(tensions[j], points[i].Tensions[j], 9);
            }
        }
    }

    [Fact]
    public void TheLockedElbowPushesBackAgainstABend()
    {
        double[] offsets = [-0.1, -0.02, 0, 0.02, 0.1];

        var points = EffortSweep.Bend(TopologyFiles.ToDocument(Wing.Locked()), LockingWing.Elbow, WingJoints, offsets);

        // Zero effort straight; holding a bend takes a torque the same way as
        // the bend, because the tendon and gravity push the joint back.
        Assert.InRange(Math.Abs(points[2].Torques[1]), 0.0, 1e-10);
        foreach (var point in points.Where(p => p.Offset != 0))
        {
            Assert.Equal(Math.Sign(point.Offset), Math.Sign(point.Torques[1]));
        }

        Assert.True(points[0].Torques[1] < points[1].Torques[1], "Effort should grow with the bend.");
        Assert.True(points[4].Torques[1] > points[3].Torques[1], "Effort should grow with the bend.");
    }

    [Fact]
    public void BendingAGroundedEndTurnsTheOtherSide()
    {
        // A pin whose end A is the ground and end B the bar: bending turns
        // end A by +Δ, which means turning the bar by −Δ.
        var bar = PlanarLink.SlenderBar("bar", mass: 2.0, length: 1.2);
        var system = new PlanarSystem([bar], PlanarSystem.StandardGravity, [new PinJoint("pivot", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("bar", bar.EndA))]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0.6, 0), 0), default);
        var document = TopologyMapping.ToDocument(system, state.Values, "pivoted_bar");

        var point = EffortSweep.Bend(document, "pivot", ["pivot"], [Math.PI / 3]).Single();

        // The bar hangs 60° below horizontal: weight moment m·g·(ℓ/2)·cos 60°,
        // clockwise. End B (the bar) must get +m·g·(ℓ/2)·cos 60°, which is
        // −τ, so τ on end A is −m·g·(ℓ/2)·cos 60°.
        Assert.Equal(Math.PI / 3, point.JointAngle, 14);
        Assert.Equal(-2.0 * 9.81 * 0.6 * 0.5, point.Torques[0], 12);
    }

    [Fact]
    public void AJointInAClosedLoopCannotBeBentAlone()
    {
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
        var document = TopologyMapping.ToDocument(system, state.Values, "triangle");

        var error = Assert.Throws<ArgumentException>(() => EffortSweep.Bend(document, "knee", ["root"], [0.1]));
        Assert.Contains("closed loop", error.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentException>(() => EffortSweep.Bend(document, "ankle", ["root"], [0.1]));
    }
}
