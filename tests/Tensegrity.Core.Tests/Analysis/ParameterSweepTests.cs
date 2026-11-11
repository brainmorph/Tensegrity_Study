using Tensegrity.Core.Analysis;
using Tensegrity.Core.Output;
using Tensegrity.RunFormat;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>
/// Parameter sweeps (PLAN.md M6): which configurations of cable rest
/// lengths and attachment points lock, and the effort each needs.
/// </summary>
public class ParameterSweepTests
{
    private const double Dt = 1e-4;

    private static readonly string[] WingJoints = [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist];

    private static TopologyDocument StillWing() => TopologyFiles.ToDocument(Wing.Locked());

    private static SweepSettings Settings(double duration, IReadOnlyList<string>? pins = null) =>
        new(Dt, duration, Math.PI / 2, pins ?? WingJoints);

    private static SweepAxis Axis(string spec, params double[] values) => new(SweepParameter.Parse(spec), values);

    [Theory]
    [InlineData(0, LockingWing.ShoulderTendon, LockingWing.Shoulder)]
    [InlineData(1, LockingWing.ElbowTendon, LockingWing.Elbow)]
    [InlineData(2, LockingWing.WristTendon, LockingWing.Wrist)]
    public void TheWingLocksShortOfEachFoldAndCollapsesAtThatJointPastIt(int tendon, string cable, string joint)
    {
        // Past the statics fold (test 10) no stable pose is left, so the
        // wing must collapse whatever its release. Released from straight
        // it swings toward its drooped rest pose and overshoots, so the
        // check short of the fold leaves room: 0.02 in scale.
        var fold = ChainStatics.ForWing(LockingWing.Create()).Fold(tendon).Scale;

        var points = ParameterSweep.Run(StillWing(), [Axis($"rest:{cable}", fold - 0.02, fold + 0.002)], Settings(2.0));

        Assert.True(points[0].Locked, $"Short of the fold the wing collapsed at {joint} at {points[0].CollapseTime} s.");
        Assert.InRange(points[0].MaxDeviation, 0.1, Math.PI / 4);
        Assert.False(points[1].Locked, $"Past the fold the wing held, bending at most {points[1].MaxDeviation * 180 / Math.PI}°.");
        Assert.Equal(joint, points[1].CollapsedJoint);
        Assert.True(double.IsNaN(points[1].FinalDeviation));
    }

    [Fact]
    public void EffortMatchesTheEffortSweep()
    {
        double[] scales = [0.99, 1.0, 1.004, 1.02];

        var points = ParameterSweep.Run(StillWing(), [Axis($"rest:{LockingWing.WristTendon}", scales)], Settings(1e-3));
        var effort = EffortSweep.Run(StillWing(), LockingWing.WristTendon, WingJoints, scales);

        for (var i = 0; i < scales.Length; i++)
        {
            Assert.Equal([scales[i]], points[i].Values);
            Assert.Equal(effort[i].Torques, points[i].Torques);
            Assert.Equal(effort[i].Tension, points[i].Tensions[2]);
        }
    }

    [Fact]
    public void AnAnchorMoveKeepsTheTendonsStretch()
    {
        double[] offsets = [-0.01, 0, 0.02];

        var points = ParameterSweep.Run(StillWing(), [Axis($"anchor:{LockingWing.WristTendon}.a.y", offsets)], Settings(1e-3));

        // Same tension in the starting pose, but a longer moment arm: raising
        // the post over-lifts the hand, so the wrist must pull it down.
        foreach (var point in points)
        {
            Assert.Equal(points[1].Tensions[2], point.Tensions[2], 9);
        }

        Assert.InRange(Math.Abs(points[1].Torques[2]), 0.0, 1e-10);
        Assert.True(points[0].Torques[2] > 0, "A lowered post should leave the hand needing lift.");
        Assert.True(points[2].Torques[2] < 0, "A raised post should over-lift the hand.");
    }

    [Fact]
    public void AnAnchorOffsetMovesTheNamedCoordinate()
    {
        var document = StillWing();
        var wrist = document.Cables.Single(c => c.Id == LockingWing.WristTendon);
        var before = (double[])wrist.B.Point!.Clone();
        var rest = wrist.RestLength;

        // The insertion sits outboard of the post, so −x moves it toward the post.
        SweepParameter.Parse($"anchor:{LockingWing.WristTendon}.b.x").Apply(document, -0.01);

        Assert.Equal(before[0] - 0.01, wrist.B.Point![0]);
        Assert.Equal(before[1], wrist.B.Point[1]);
        Assert.True(wrist.RestLength < rest, "Moving the hand insertion toward the post shortens the tendon.");
    }

    [Fact]
    public void TwoAxesMakeAGridWithTheFirstVaryingSlowest()
    {
        var points = ParameterSweep.Run(
            StillWing(),
            [Axis($"rest:{LockingWing.WristTendon}", 1.0, 1.01), Axis($"anchor:{LockingWing.ElbowTendon}.a.x", -0.01, 0, 0.01)],
            Settings(1e-3));

        Assert.Equal(6, points.Count);
        Assert.Equal([1.0, -0.01], points[0].Values);
        Assert.Equal([1.0, 0.01], points[2].Values);
        Assert.Equal([1.01, -0.01], points[3].Values);
        Assert.All(points, p => Assert.True(p.Locked));
    }

    [Fact]
    public void ATopologyWithoutPinsIsJudgedOnItsLinkAngles()
    {
        // The free planar X, floating without gravity, rings with its
        // opposite initial twists but holds its shape.
        var document = TopologyFiles.ToDocument(ReferenceTopologies.Create("planar_x_free"));

        var point = ParameterSweep.Run(document, [Axis($"rest:{document.Cables[0].Id}", 1.0)], Settings(0.05, pins: [])).Single();

        Assert.Empty(point.Torques);
        Assert.True(point.Locked);
        Assert.Null(point.CollapsedJoint);
        Assert.InRange(point.MaxDeviation, 1e-5, 0.01);
    }

    [Fact]
    public void BadSweepsFailBeforeRunning()
    {
        Assert.Throws<ArgumentException>(() => ParameterSweep.Run(StillWing(), [Axis("rest:nope", 1.0)], Settings(1e-3)));
        Assert.Throws<ArgumentException>(() => ParameterSweep.Run(StillWing(), [], Settings(1e-3)));
        Assert.Throws<ArgumentException>(() => ParameterSweep.Run(StillWing(), [Axis($"rest:{LockingWing.WristTendon}", 1.0)], Settings(1.5e-4)));
        Assert.Throws<ArgumentException>(() => ParameterSweep.Run(StillWing(), [Axis($"rest:{LockingWing.WristTendon}", 1.0)], Settings(1e-3, ["ankle"])));
    }

    [Theory]
    [InlineData("rest:")]
    [InlineData("rest")]
    [InlineData("anchor:wrist_tendon.c.x")]
    [InlineData("anchor:wrist_tendon.a.z")]
    [InlineData("anchor:wrist_tendon.a")]
    [InlineData("length:wrist_tendon")]
    public void MalformedSpecsAreRejected(string spec) =>
        Assert.Throws<FormatException>(() => SweepParameter.Parse(spec));

    [Fact]
    public void ValuesAreARangeOrAList()
    {
        Assert.Equal([1.0, 1.05, 1.1], ParameterSweep.ParseValues("1:1.1:3").Select(v => Math.Round(v, 12)));
        Assert.Equal(1.1, ParameterSweep.ParseValues("1:1.1:3")[2]); // The end is exact.
        Assert.Equal([0.5], ParameterSweep.ParseValues("0.5:0.5:1"));
        Assert.Equal([1.0, -2.0, 0.25], ParameterSweep.ParseValues("1,-2,0.25"));
        Assert.Throws<FormatException>(() => ParameterSweep.ParseValues("1:2:1"));
        Assert.Throws<FormatException>(() => ParameterSweep.ParseValues("1:2:x"));
        Assert.Throws<FormatException>(() => ParameterSweep.ParseValues("1,two"));
    }
}
