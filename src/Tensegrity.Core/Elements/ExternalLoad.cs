using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Elements;

/// <summary>A breakpoint of an <see cref="ExternalLoad"/>: the force (world frame) at a time.</summary>
public readonly record struct LoadPoint(double Time, double Fx, double Fy);

/// <summary>
/// A perturbation (PLAN.md M6): a force given in the world frame, applied
/// at a body-fixed point on a link, and piecewise-linear in time through
/// <see cref="Points"/> (held before the first and after the last). It does
/// work F·v at the point's velocity, which the system integrates so the
/// energy balance still closes.
///
/// An impulse is modeled as a short force pulse (<see cref="Pulse"/>) rather
/// than a jump in velocity: the trajectory stays continuous, pin constraints
/// need no separate impulse solve, and real impacts last a finite time. The
/// pulse's breakpoints are split like any schedule's, so RK4 keeps its order.
/// </summary>
public sealed class ExternalLoad
{
    private readonly LoadPoint[] points;

    public ExternalLoad(string id, string linkId, Vector2d point, IEnumerable<LoadPoint> schedule)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentException.ThrowIfNullOrWhiteSpace(linkId);
        ArgumentNullException.ThrowIfNull(schedule);
        Id = id;
        Attachment = Attachment.OnLink(linkId, point);
        points = schedule.ToArray();

        // Validates times and values, and gives each component its segments.
        Fx = new Schedule(points.Select(p => new SchedulePoint(p.Time, p.Fx)));
        Fy = new Schedule(points.Select(p => new SchedulePoint(p.Time, p.Fy)));
    }

    public string Id { get; }

    /// <summary>The point the force acts at, in its link's body frame.</summary>
    public Attachment Attachment { get; }

    /// <summary>The breakpoints in time order.</summary>
    public IReadOnlyList<LoadPoint> Points => points;

    /// <summary>World x component of the force over time, N.</summary>
    public Schedule Fx { get; }

    /// <summary>World y component of the force over time, N.</summary>
    public Schedule Fy { get; }

    /// <summary>The force at <paramref name="time"/>, N.</summary>
    public Vector2d ForceAt(double time) => new(Fx.ValueAt(time), Fy.ValueAt(time));

    /// <summary>
    /// A triangular force pulse delivering <paramref name="impulse"/> (N·s,
    /// world frame) at a point on a link, starting at
    /// <paramref name="start"/> and lasting <paramref name="duration"/>: zero,
    /// rising linearly to its peak 2·J/τ at mid-pulse, and back to zero.
    /// </summary>
    public static ExternalLoad Pulse(string id, string linkId, Vector2d point, Vector2d impulse, double start, double duration)
    {
        if (!double.IsFinite(duration) || duration <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(duration), duration, "Must be finite and positive.");
        }

        var peak = 2.0 / duration * impulse;
        return new ExternalLoad(
            id,
            linkId,
            point,
            [
                new LoadPoint(start, 0, 0),
                new LoadPoint(start + (duration / 2), peak.X, peak.Y),
                new LoadPoint(start + duration, 0, 0),
            ]);
    }
}
