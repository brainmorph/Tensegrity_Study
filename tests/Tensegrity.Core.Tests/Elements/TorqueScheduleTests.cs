using Tensegrity.Core.Elements;

namespace Tensegrity.Core.Tests.Elements;

/// <summary>The actuator torque schedule on its own: values, segments and validation.</summary>
public class TorqueScheduleTests
{
    // 0 until t = 1, ramps to 4 at t = 3, falls to −2 at t = 4, then holds.
    private static readonly TorqueSchedule Ramp = new(
        [new TorquePoint(1, 0), new TorquePoint(3, 4), new TorquePoint(4, -2)]);

    [Fact]
    public void ConstantScheduleHasOneSegmentAndTheSameTorqueEverywhere()
    {
        var schedule = TorqueSchedule.Constant(2.5);

        Assert.True(schedule.IsConstant);
        Assert.Equal(1, schedule.SegmentCount);
        foreach (var t in new[] { -10.0, 0.0, 1e9 })
        {
            Assert.Equal(0, schedule.SegmentAt(t));
            Assert.Equal(2.5, schedule.TorqueAt(t));
        }
    }

    [Fact]
    public void TorqueIsPiecewiseLinearAndHeldOutsideTheBreakpoints()
    {
        Assert.Equal(0.0, Ramp.TorqueAt(-5));
        Assert.Equal(0.0, Ramp.TorqueAt(1));
        Assert.Equal(2.0, Ramp.TorqueAt(2), 15);
        Assert.Equal(4.0, Ramp.TorqueAt(3));
        Assert.Equal(1.0, Ramp.TorqueAt(3.5), 15);
        Assert.Equal(-2.0, Ramp.TorqueAt(4));
        Assert.Equal(-2.0, Ramp.TorqueAt(100));
    }

    [Fact]
    public void SegmentCountsTheBreakpointsAtOrBeforeTheTime()
    {
        Assert.Equal(4, Ramp.SegmentCount);
        Assert.Equal(0, Ramp.SegmentAt(0.999));
        Assert.Equal(1, Ramp.SegmentAt(1));
        Assert.Equal(1, Ramp.SegmentAt(2.999));
        Assert.Equal(2, Ramp.SegmentAt(3));
        Assert.Equal(3, Ramp.SegmentAt(4));
        Assert.Equal(3, Ramp.SegmentAt(1e9));
    }

    [Fact]
    public void SegmentFormulaExtendsPastTheSegmentEnds()
    {
        // Segment 1 is the 0 → 4 ramp over [1, 3]: slope 2.
        Assert.Equal(4.2, Ramp.TorqueInSegment(1, 3.1), 14);
        Assert.Equal(-0.2, Ramp.TorqueInSegment(1, 0.9), 14);

        // The end segments are flat.
        Assert.Equal(0.0, Ramp.TorqueInSegment(0, 1.5));
        Assert.Equal(-2.0, Ramp.TorqueInSegment(3, 3.9));
        Assert.Throws<ArgumentOutOfRangeException>(() => Ramp.TorqueInSegment(4, 0));
    }

    [Fact]
    public void RejectsBadSchedules()
    {
        Assert.Throws<ArgumentException>(() => new TorqueSchedule([]));
        Assert.Throws<ArgumentException>(() => new TorqueSchedule([new TorquePoint(1, 0), new TorquePoint(1, 2)]));
        Assert.Throws<ArgumentException>(() => new TorqueSchedule([new TorquePoint(2, 0), new TorquePoint(1, 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => new TorqueSchedule([new TorquePoint(double.NaN, 0)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => TorqueSchedule.Constant(double.PositiveInfinity));
        Assert.Throws<ArgumentException>(() => new JointActuator(" ", "pin", Ramp));
        Assert.Throws<ArgumentException>(() => new JointActuator("a", "", Ramp));
    }
}
