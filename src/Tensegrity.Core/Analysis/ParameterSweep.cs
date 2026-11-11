using System.Globalization;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Analysis;

/// <summary>
/// One quantity a <see cref="ParameterSweep"/> varies, parsed from a spec:
/// <list type="bullet">
/// <item><c>rest:CABLE</c>: the cable's rest length, as a multiple of the
/// topology's own.</item>
/// <item><c>anchor:CABLE.a.x</c> (or <c>.a.y</c>, <c>.b.x</c>, <c>.b.y</c>):
/// an offset in metres added to one coordinate of the cable's end A or B,
/// in its link's body frame (or the world, for a ground end). The rest
/// length is scaled with the cable's length in the initial pose, so its
/// stretch, and with it the initial tension, stays as designed: the axis
/// changes the geometry, not the pretension.</item>
/// </list>
/// </summary>
public sealed record SweepParameter(string Spec, string Cable, bool IsAnchor, bool EndA, bool AlongX)
{
    public static SweepParameter Parse(string spec)
    {
        ArgumentNullException.ThrowIfNull(spec);
        var colon = spec.IndexOf(':', StringComparison.Ordinal);
        var kind = colon < 0 ? string.Empty : spec[..colon];
        var target = colon < 0 ? string.Empty : spec[(colon + 1)..];
        if (kind == "rest" && target.Length > 0)
        {
            return new SweepParameter(spec, target, IsAnchor: false, EndA: false, AlongX: false);
        }

        var parts = target.Split('.');
        if (kind == "anchor" && parts.Length == 3 && parts[0].Length > 0 && parts[1] is "a" or "b" && parts[2] is "x" or "y")
        {
            return new SweepParameter(spec, parts[0], IsAnchor: true, EndA: parts[1] == "a", AlongX: parts[2] == "x");
        }

        throw new FormatException($"Sweep parameter '{spec}' is not rest:CABLE or anchor:CABLE.(a|b).(x|y).");
    }

    /// <summary>Sets this parameter to <paramref name="value"/> in <paramref name="document"/>.</summary>
    public void Apply(TopologyDocument document, double value)
    {
        ArgumentNullException.ThrowIfNull(document);
        var cable = document.Cables.Find(c => string.Equals(c.Id, Cable, StringComparison.Ordinal))
            ?? throw new ArgumentException($"Topology '{document.Name}' has no cable '{Cable}'.");
        if (!IsAnchor)
        {
            cable.RestLength *= value;
            return;
        }

        var before = Length(document, cable);
        var end = EndA ? cable.A : cable.B;
        var coordinates = end.IsGround ? end.Ground! : end.Point!;
        coordinates[AlongX ? 0 : 1] += value;
        var after = Length(document, cable);
        if (!(before > 0) || !(after > 0))
        {
            throw new ArgumentException($"Cable '{Cable}' has zero length, so '{Spec}' cannot keep its stretch.");
        }

        cable.RestLength *= after / before;
    }

    private static double Length(TopologyDocument document, CableDocument cable)
    {
        var a = EffortSweep.WorldPoint(document, cable.A);
        var b = EffortSweep.WorldPoint(document, cable.B);
        return Math.Sqrt(((b.X - a.X) * (b.X - a.X)) + ((b.Y - a.Y) * (b.Y - a.Y)));
    }

    public override string ToString() => Spec;
}

/// <summary>One axis of a sweep grid: a parameter and the values it takes.</summary>
public sealed record SweepAxis(SweepParameter Parameter, IReadOnlyList<double> Values);

/// <summary>How each configuration in a <see cref="ParameterSweep"/> is run and judged.</summary>
/// <param name="Dt">Step size, s.</param>
/// <param name="Duration">Run length, s.</param>
/// <param name="CollapseAngle">
/// A configuration collapses when any joint angle moves further than this
/// from where it started, rad. A topology without pins uses its links' angles.
/// </param>
/// <param name="EffortPins">Pins whose holding torques in the starting pose are reported.</param>
/// <param name="EventTolerance">Bracket width for cable and contact mode switches, s.</param>
public sealed record SweepSettings(
    double Dt,
    double Duration,
    double CollapseAngle,
    IReadOnlyList<string> EffortPins,
    double EventTolerance = EventSplittingStepper.DefaultTimeTolerance);

/// <summary>One configuration of a <see cref="ParameterSweep"/>.</summary>
/// <param name="Values">The value of each axis, in axis order.</param>
/// <param name="Locked">True if no joint moved past the collapse angle for the whole run.</param>
/// <param name="MaxDeviation">Largest joint-angle change from the start over the run (up to the collapse), rad.</param>
/// <param name="FinalDeviation">Largest joint-angle change at the end of the run, rad; NaN after a collapse, which stops the run.</param>
/// <param name="CollapseTime">When a joint first passed the collapse angle, s; null if locked.</param>
/// <param name="CollapsedJoint">The pin (or, without pins, the link) that passed it; null if locked.</param>
/// <param name="Tensions">Every cable's tension in the starting pose at rest, N, in the topology's cable order.</param>
/// <param name="Torques">Holding torque at each effort pin in the starting pose at rest, N·m.</param>
/// <param name="UnheldAcceleration">Acceleration the effort pins cannot cancel in that pose (largest of m/s² and rad/s²).</param>
/// <param name="Redundancy">Torque combinations at the effort pins that move nothing.</param>
public sealed record SweepPoint(
    IReadOnlyList<double> Values,
    bool Locked,
    double MaxDeviation,
    double FinalDeviation,
    double? CollapseTime,
    string? CollapsedJoint,
    IReadOnlyList<double> Tensions,
    IReadOnlyList<double> Torques,
    double UnheldAcceleration,
    int Redundancy);

/// <summary>
/// Parameter sweeps (PLAN.md M6): runs a topology over a grid of cable rest
/// lengths and attachment points and records which configurations lock and
/// how much effort each needs.
///
/// Each configuration is run from the topology's own initial state,
/// including any initial velocities and loads, so "locked" means it holds
/// that release or perturbation, not merely that a stable rest pose exists
/// (the statics answer, test 10). A configuration released from a pose
/// that is no longer its rest pose swings toward the new one, and can
/// overshoot past a fold the statics would survive. The effort is test
/// 11's: the holding torque that keeps the starting pose still, at rest.
///
/// Configurations are independent and deterministic, so they run in
/// parallel and give the same results on any number of threads.
/// </summary>
public static class ParameterSweep
{
    /// <summary>
    /// Every combination of the axes' values, the first axis varying
    /// slowest, run and judged with <paramref name="settings"/>.
    /// </summary>
    public static IReadOnlyList<SweepPoint> Run(TopologyDocument document, IReadOnlyList<SweepAxis> axes, SweepSettings settings)
    {
        ArgumentNullException.ThrowIfNull(document);
        ArgumentNullException.ThrowIfNull(axes);
        ArgumentNullException.ThrowIfNull(settings);
        if (axes.Count == 0 || axes.Any(a => a.Values.Count == 0))
        {
            throw new ArgumentException("A sweep needs at least one axis, each with at least one value.", nameof(axes));
        }

        var steps = Math.Round(settings.Duration / settings.Dt);
        if (!(settings.Dt > 0) || !(steps >= 1) || Math.Abs((steps * settings.Dt) - settings.Duration) > 1e-9 * settings.Duration)
        {
            throw new ArgumentException($"Duration {settings.Duration} s must be a whole number of {settings.Dt} s steps.", nameof(settings));
        }

        if (!(settings.CollapseAngle > 0))
        {
            throw new ArgumentException("The collapse angle must be positive.", nameof(settings));
        }

        var grid = Grid(axes);

        // Configure every point and solve its effort first, so a bad
        // parameter or pin fails here, before any run, with its own exception.
        var documents = grid.Select(values => Configure(document, axes, values)).ToArray();
        var efforts = documents.Select(d => EffortSweep.Solve(EffortSweep.AtRest(d), settings.EffortPins)).ToArray();

        var points = new SweepPoint[grid.Count];
        Parallel.For(0, grid.Count, i => points[i] = RunOne(documents[i], grid[i], efforts[i].Tensions, efforts[i].Hold, (long)steps, settings));
        return points;
    }

    private static List<double[]> Grid(IReadOnlyList<SweepAxis> axes)
    {
        var grid = new List<double[]> { Array.Empty<double>() };
        foreach (var axis in axes)
        {
            grid = grid.SelectMany(prefix => axis.Values.Select(v => (double[])[.. prefix, v])).ToList();
        }

        return grid;
    }

    private static TopologyDocument Configure(TopologyDocument document, IReadOnlyList<SweepAxis> axes, double[] values)
    {
        var copy = EffortSweep.Copy(document);
        for (var k = 0; k < axes.Count; k++)
        {
            axes[k].Parameter.Apply(copy, values[k]);
        }

        return copy;
    }

    private static SweepPoint RunOne(TopologyDocument document, double[] values, double[] tensions, HoldingTorques hold, long steps, SweepSettings settings)
    {
        var (system, initial) = TopologyMapping.FromDocument(document);
        var state = new SystemState(initial);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system, settings.EventTolerance);
        var usePins = system.Pins.Count > 0;
        var count = usePins ? system.Pins.Count : system.BodyCount;
        var start = new double[count];
        Angles(system, state.Values, usePins, start);

        var now = new double[count];
        var maxDeviation = 0.0;
        double deviation = 0;
        double? collapse = null;
        string? collapsed = null;
        for (long k = 1; k <= steps && collapse is null; k++)
        {
            stepper.Step(state, settings.Dt);
            state.Time = k * settings.Dt;
            Angles(system, state.Values, usePins, now);
            deviation = 0;
            for (var j = 0; j < count; j++)
            {
                var change = Math.Abs(now[j] - start[j]);

                // NaN fails the comparison too, so a run that blows up counts as collapsed.
                if (!(change <= settings.CollapseAngle))
                {
                    collapse = state.Time;
                    collapsed = usePins ? system.Pins[j].Id : system.Links[j].Id;
                    break;
                }

                deviation = Math.Max(deviation, change);
            }

            maxDeviation = Math.Max(maxDeviation, deviation);
        }

        return new SweepPoint(
            values,
            collapse is null,
            maxDeviation,
            collapse is null ? deviation : double.NaN,
            collapse,
            collapsed,
            tensions,
            hold.Torques,
            Math.Max(hold.MaxLinearAcceleration, hold.MaxAngularAcceleration),
            hold.Redundancy);
    }

    private static void Angles(PlanarSystem system, ReadOnlySpan<double> state, bool usePins, Span<double> angles)
    {
        for (var j = 0; j < angles.Length; j++)
        {
            angles[j] = usePins ? system.GetJointAngle(state, j).Angle : system.GetPose(state, j).Angle;
        }
    }

    /// <summary>Parses <c>FROM:TO:N</c> into N evenly spaced values, or a comma-separated list.</summary>
    public static IReadOnlyList<double> ParseValues(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        var range = text.Split(':');
        if (range.Length == 3)
        {
            var from = Number(range[0]);
            var to = Number(range[1]);
            if (!int.TryParse(range[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var n) || n < 1 || (n == 1 && from != to))
            {
                throw new FormatException($"'{text}' needs a count of at least 2 (or 1 with FROM = TO).");
            }

            return n == 1 ? [from] : Enumerable.Range(0, n).Select(i => i == n - 1 ? to : from + ((to - from) * i / (n - 1))).ToArray();
        }

        return text.Split(',').Select(Number).ToArray();

        double Number(string s) =>
            double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && double.IsFinite(v)
                ? v
                : throw new FormatException($"'{s}' in '{text}' is not a number.");
    }
}
