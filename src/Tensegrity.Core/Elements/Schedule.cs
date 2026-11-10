namespace Tensegrity.Core.Elements;

/// <summary>A breakpoint of a <see cref="Schedule"/>: a value at a time.</summary>
public readonly record struct SchedulePoint(double Time, double Value);

/// <summary>
/// A quantity as a function of time, such as an actuator torque:
/// piecewise-linear through a list of breakpoints, held constant before the
/// first and after the last. A single breakpoint is a constant.
///
/// The value has a kink at every breakpoint, where RK4 would lose its
/// order. Each stretch between breakpoints is therefore a <em>segment</em>,
/// and the segment is a mode like a cable's slack or taut mode: the system
/// latches it at the start of a step, evaluates the segment's own linear
/// formula for the whole step, and the stepper splits the step at the
/// breakpoint (PLAN.md 4.2).
///
/// Segment k holds the times with exactly k breakpoints at or before them,
/// so segment 0 is t &lt; t₀ and segment n is t ≥ tₙ₋₁. A constant schedule
/// has a single segment, 0.
/// </summary>
public sealed class Schedule
{
    private readonly SchedulePoint[] points;

    public Schedule(IEnumerable<SchedulePoint> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        this.points = points.ToArray();
        if (this.points.Length == 0)
        {
            throw new ArgumentException("A schedule needs at least one point.", nameof(points));
        }

        for (var i = 0; i < this.points.Length; i++)
        {
            var p = this.points[i];
            if (!double.IsFinite(p.Time) || !double.IsFinite(p.Value))
            {
                throw new ArgumentOutOfRangeException(nameof(points), p, "Times and values must be finite.");
            }

            if (i > 0 && !(p.Time > this.points[i - 1].Time))
            {
                throw new ArgumentException("Schedule times must be strictly increasing.", nameof(points));
            }
        }
    }

    /// <summary>A value that never changes.</summary>
    public static Schedule Constant(double value) => new([new SchedulePoint(0, value)]);

    /// <summary>The breakpoints in time order.</summary>
    public IReadOnlyList<SchedulePoint> Points => points;

    /// <summary>True if the value is the same at all times, so there is nothing to latch.</summary>
    public bool IsConstant => points.Length == 1;

    /// <summary>Number of segments: one for a constant schedule, else breakpoints + 1.</summary>
    public int SegmentCount => IsConstant ? 1 : points.Length + 1;

    /// <summary>The segment that holds time <paramref name="time"/>.</summary>
    public int SegmentAt(double time)
    {
        if (IsConstant)
        {
            return 0;
        }

        var k = 0;
        while (k < points.Length && points[k].Time <= time)
        {
            k++;
        }

        return k;
    }

    /// <summary>Value at <paramref name="time"/>.</summary>
    public double ValueAt(double time) => ValueInSegment(SegmentAt(time), time);

    /// <summary>
    /// Value from segment <paramref name="segment"/>'s formula at
    /// <paramref name="time"/>, extended past the segment's ends. RK4 stages
    /// that land just outside the latched segment use this, so the
    /// right-hand side stays smooth over the step.
    /// </summary>
    public double ValueInSegment(int segment, double time)
    {
        if ((uint)segment >= (uint)SegmentCount)
        {
            throw new ArgumentOutOfRangeException(nameof(segment), segment, $"Schedule has {SegmentCount} segments.");
        }

        if (segment == 0)
        {
            return points[0].Value;
        }

        if (segment == points.Length)
        {
            return points[^1].Value;
        }

        var a = points[segment - 1];
        var b = points[segment];
        return a.Value + ((b.Value - a.Value) * (time - a.Time) / (b.Time - a.Time));
    }
}
