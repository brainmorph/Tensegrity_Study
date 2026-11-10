using Tensegrity.Core.Elements;

namespace Tensegrity.Core.Tests.Elements;

/// <summary>The actuator torque schedule on its own: values, segments and validation.</summary>
public class ScheduleTests
{
    // 0 until t = 1, ramps to 4 at t = 3, falls to −2 at t = 4, then holds.
    private static readonly Schedule Ramp = new(
        [new SchedulePoint(1, 0), new SchedulePoint(3, 4), new SchedulePoint(4, -2)]);

    [Fact]
    public void ConstantScheduleHasOneSegmentAndTheSameTorqueEverywhere()
    {
        var schedule = Schedule.Constant(2.5);

        Assert.True(schedule.IsConstant);
        Assert.Equal(1, schedule.SegmentCount);
        foreach (var t in new[] { -10.0, 0.0, 1e9 })
        {
            Assert.Equal(0, schedule.SegmentAt(t));
            Assert.Equal(2.5, schedule.ValueAt(t));
        }
    }

    [Fact]
    public void TorqueIsPiecewiseLinearAndHeldOutsideTheBreakpoints()
    {
        Assert.Equal(0.0, Ramp.ValueAt(-5));
        Assert.Equal(0.0, Ramp.ValueAt(1));
        Assert.Equal(2.0, Ramp.ValueAt(2), 15);
        Assert.Equal(4.0, Ramp.ValueAt(3));
        Assert.Equal(1.0, Ramp.ValueAt(3.5), 15);
        Assert.Equal(-2.0, Ramp.ValueAt(4));
        Assert.Equal(-2.0, Ramp.ValueAt(100));
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
        Assert.Equal(4.2, Ramp.ValueInSegment(1, 3.1), 14);
        Assert.Equal(-0.2, Ramp.ValueInSegment(1, 0.9), 14);

        // The end segments are flat.
        Assert.Equal(0.0, Ramp.ValueInSegment(0, 1.5));
        Assert.Equal(-2.0, Ramp.ValueInSegment(3, 3.9));
        Assert.Throws<ArgumentOutOfRangeException>(() => Ramp.ValueInSegment(4, 0));
    }

    [Fact]
    public void RejectsBadSchedules()
    {
        Assert.Throws<ArgumentException>(() => new Schedule([]));
        Assert.Throws<ArgumentException>(() => new Schedule([new SchedulePoint(1, 0), new SchedulePoint(1, 2)]));
        Assert.Throws<ArgumentException>(() => new Schedule([new SchedulePoint(2, 0), new SchedulePoint(1, 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Schedule([new SchedulePoint(double.NaN, 0)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => Schedule.Constant(double.PositiveInfinity));
        Assert.Throws<ArgumentException>(() => new JointActuator(" ", "pin", Ramp));
        Assert.Throws<ArgumentException>(() => new JointActuator("a", "", Ramp));
    }
}
