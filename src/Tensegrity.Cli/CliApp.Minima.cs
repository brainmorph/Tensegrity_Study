using System.Globalization;
using System.Text;
using Tensegrity.Core.Analysis;
using Tensegrity.RunFormat;
using Tensegrity.Topology;

namespace Tensegrity.Cli;

/// <summary>The <c>minima</c> command: statics of the tendon-held five-bar.</summary>
public sealed partial class CliApp
{
    private int MinimaCommand(Options options)
    {
        options.Allow("set", "vary", "grid", "map", "save", "at", "csv");
        var geometry = new FiveBarGeometry();
        try
        {
            foreach (var set in options.All("set"))
            {
                geometry = FiveBarParameter.ApplyAll(geometry, set);
            }
        }
        catch (FormatException e)
        {
            throw new UsageException($"--set: {e.Message}");
        }

        var varies = options.All("vary");
        if (varies.Count > 2)
        {
            throw new UsageException("minima takes at most two --vary options.");
        }

        var axes = varies.Select(ParseFiveBarAxis).ToArray();
        var gridDegrees = options.Double("grid") ?? (axes.Length > 0 ? 10 : 5);
        if (!(gridDegrees > 0) || gridDegrees > 90)
        {
            throw new UsageException("--grid must be in (0, 90] degrees.");
        }

        var map = options.Value("map") switch
        {
            null or "no" or "false" => false,
            "yes" or "true" => true,
            var other => throw new UsageException($"--map '{other}' must be yes or no."),
        };
        var save = options.Value("save");
        var at = options.Int("at");
        if ((save is null) != (at is null))
        {
            throw new UsageException("--save and --at go together.");
        }

        if (axes.Length > 0 && (save is not null || map))
        {
            throw new UsageException("--save and --map need a single geometry, not --vary.");
        }

        var settings = new FiveBarSettings(GridStep: gridDegrees * Math.PI / 180);
        return axes.Length == 0
            ? MinimaSingle(geometry, settings, map, save, at, options.Value("csv"))
            : MinimaSweep(geometry, settings, axes, options.Value("csv"));

        static (FiveBarParameter Parameter, IReadOnlyList<double> Values) ParseFiveBarAxis(string vary)
        {
            var equals = vary.IndexOf('=', StringComparison.Ordinal);
            if (equals < 0)
            {
                throw new UsageException($"--vary '{vary}' needs NAME=VALUES.");
            }

            try
            {
                return (FiveBarParameter.Parse(vary[..equals]), ParameterSweep.ParseValues(vary[(equals + 1)..]));
            }
            catch (FormatException e)
            {
                throw new UsageException($"--vary '{vary}': {e.Message}");
            }
        }
    }

    private int MinimaSingle(FiveBarGeometry geometry, FiveBarSettings settings, bool map, string? save, int? at, string? csv)
    {
        var statics = new FiveBarStatics(geometry);
        var report = statics.Analyze(settings);
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"minima: five-bar, {report.Starts} starts on a {Deg(settings.GridStep):G4} deg grid in servo angle {Deg(settings.ServoMin):G4} to {Deg(settings.ServoMax):G4} deg, both sides of E"));
        stdout.WriteLine(Describe(geometry));
        stdout.WriteLine();
        stdout.WriteLine("| # | servo (deg) | passive (deg) | E side | energy (J) | slowest mode (Hz) | tension C, D, E, CE (N) | starts | in range |");
        stdout.WriteLine("|---|---|---|---|---|---|---|---|---|");
        for (var i = 0; i < report.Minima.Count; i++)
        {
            var m = report.Minima[i];
            var tensions = string.Join(", ", m.Tensions.Select(t => t.ToString("F2", CultureInfo.InvariantCulture)));
            stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {Label(i, m.InRange)} | {Deg(m.Pose.Servo):F2} | {Deg(m.Pose.Passive):F2} | {(m.Side > 0 ? "left" : "right")} | {m.Energy:F5} | {m.SlowestFrequency:F2} | {tensions} | {m.BasinStarts} | {(m.InRange ? "yes" : "no")} |"));
        }

        if (report.Transitions.Count > 0)
        {
            stdout.WriteLine();
            stdout.WriteLine($"transitions, servo held in {Deg(settings.PathStep):G3} deg steps and the rest relaxed:");
            stdout.WriteLine();
            stdout.WriteLine("| from | to | arrived | peak torque (N m) | at (deg) | snaps at servo deg (passive jump deg) |");
            stdout.WriteLine("|---|---|---|---|---|---|");
            foreach (var t in report.Transitions)
            {
                var arrived = t.Arrived ? "yes"
                    : t.BlockedAt is { } blocked ? string.Create(CultureInfo.InvariantCulture, $"no, blocked at {Deg(blocked):F2}")
                    : t.EndedAt is { } ended ? $"no, ended at {Label(ended, report.Minima[ended].InRange)}"
                    : "no, ended at no minimum";
                var snaps = t.Snaps.Count == 0 ? "-" : string.Join(", ", t.Snaps.Select(s => string.Create(CultureInfo.InvariantCulture, $"{Deg(s.Servo):F2} ({Deg(s.Jump):+0.0;-0.0})")));
                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"| {Label(t.From, true)} | {Label(t.To, true)} | {arrived} | {t.PeakTorque:F4} | {Deg(t.PeakAt):F2} | {snaps} |"));
            }
        }

        stdout.WriteLine();
        WriteChecks(report);

        if (map)
        {
            WriteBasinMap(report);
        }

        if (save is not null)
        {
            var n = at!.Value;
            if (n < 1 || n > report.Minima.Count)
            {
                throw new UsageException($"--at {n} is not a minimum; there are {report.Minima.Count}.");
            }

            var minimum = report.Minima[n - 1];
            var description = string.Create(CultureInfo.InvariantCulture,
                $"Tendon-held five-bar at rest in minimum {Label(n - 1, minimum.InRange)} (servo {Deg(minimum.Pose.Servo):F2} deg, passive {Deg(minimum.Pose.Passive):F2} deg). {Describe(geometry)}");
            var model = FiveBar.Create(geometry, minimum.Pose, "five_bar", description, new SimulationDocument { Dt = 1e-4, Duration = 2.0, FrameStride = 100 });
            TopologyFiles.Save(model, save);
            stdout.WriteLine($"wrote {save}");
        }

        if (csv is not null)
        {
            var text = new StringBuilder("from,to,servo_rad,passive_rad,torque_n_m\n");
            foreach (var t in report.Transitions)
            {
                foreach (var p in t.Path)
                {
                    text.AppendLine(string.Create(CultureInfo.InvariantCulture, $"{t.From + 1},{t.To + 1},{p.Servo:R},{p.Passive:R},{p.Torque:R}"));
                }
            }

            File.WriteAllText(csv, text.ToString());
            stdout.WriteLine($"wrote {csv}");
        }

        return Success;
    }

    private int MinimaSweep(FiveBarGeometry geometry, FiveBarSettings settings, IReadOnlyList<(FiveBarParameter Parameter, IReadOnlyList<double> Values)> axes, string? csv)
    {
        var grid = new List<double[]> { Array.Empty<double>() };
        foreach (var axis in axes)
        {
            grid = grid.SelectMany(prefix => axis.Values.Select(v => (double[])[.. prefix, v])).ToList();
        }

        // Validate every configuration first, so a bad value fails before any analysis.
        var geometries = grid.Select(values =>
        {
            var g = geometry;
            for (var k = 0; k < axes.Count; k++)
            {
                g = axes[k].Parameter.Apply(g, values[k]);
            }

            g.Validate();
            return g;
        }).ToArray();
        var reports = new FiveBarReport[grid.Count];
        Parallel.For(0, grid.Count, i => reports[i] = new FiveBarStatics(geometries[i]).Analyze(settings));

        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"minima sweep: five-bar, {grid.Count} configurations, {Deg(settings.GridStep):G4} deg start grid"));
        stdout.WriteLine(Describe(geometry));
        stdout.WriteLine();
        var axisColumns = string.Join(" | ", axes.Select(a => a.Parameter.Name));
        stdout.WriteLine($"| {axisColumns} | in range | out of range | servo angles (deg) | min gap (deg) | range ok | transitions ok | peak torque (N m) | pass |");
        stdout.WriteLine($"|{string.Concat(axes.Select(_ => "---|"))}---|---|---|---|---|---|---|---|");
        for (var i = 0; i < grid.Count; i++)
        {
            var r = reports[i];
            var values = string.Join(" | ", grid[i].Select(v => v.ToString("G6", CultureInfo.InvariantCulture)));
            var angles = string.Join(", ", r.InRange.Select(m => Deg(m.Pose.Servo).ToString("F1", CultureInfo.InvariantCulture)));
            stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {values} | {r.InRange.Count} | {r.Minima.Count - r.InRange.Count} | {(angles.Length > 0 ? angles : "-")} | {MinGap(r)} | {YesNo(r.RangeOk)} | {YesNo(r.TransitionsOk)} | {r.PeakTorque:F3} | {(r.Passes ? "PASS" : "-")} |"));
        }

        stdout.WriteLine();
        stdout.WriteLine($"pass {reports.Count(r => r.Passes)} of {reports.Length}.");
        if (axes.Count == 2)
        {
            stdout.WriteLine();
            stdout.WriteLine($"rows {axes[0].Parameter.Name}, columns {axes[1].Parameter.Name}; the number of minima in range, * where every check passes.");
            stdout.WriteLine($"columns: {string.Join(", ", axes[1].Values.Select(v => v.ToString("G6", CultureInfo.InvariantCulture)))}");
            var columns = axes[1].Values.Count;
            for (var row = 0; row < axes[0].Values.Count; row++)
            {
                var cells = string.Concat(reports.Skip(row * columns).Take(columns).Select(r => r.Passes ? " *" : $" {Math.Min(r.InRange.Count, 9)}"));
                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {axes[0].Values[row],12:G6} |{cells}"));
            }
        }

        if (csv is not null)
        {
            var text = new StringBuilder();
            text.AppendLine(string.Join(",", [.. axes.Select(a => a.Parameter.Name), "in_range", "out_of_range", "servo_angles_rad", "range_ok", "gaps_ok", "transitions_ok", "peak_torque_n_m", "pass"]));
            for (var i = 0; i < grid.Count; i++)
            {
                var r = reports[i];
                IEnumerable<string> row =
                [
                    .. grid[i].Select(v => v.ToString("R", CultureInfo.InvariantCulture)),
                    r.InRange.Count.ToString(CultureInfo.InvariantCulture),
                    (r.Minima.Count - r.InRange.Count).ToString(CultureInfo.InvariantCulture),
                    string.Join(";", r.InRange.Select(m => m.Pose.Servo.ToString("R", CultureInfo.InvariantCulture))),
                    r.RangeOk ? "1" : "0", r.GapsOk ? "1" : "0", r.TransitionsOk ? "1" : "0",
                    r.PeakTorque.ToString("R", CultureInfo.InvariantCulture),
                    r.Passes ? "1" : "0",
                ];
                text.AppendLine(string.Join(",", row));
            }

            File.WriteAllText(csv, text.ToString());
            stdout.WriteLine($"wrote {csv}");
        }

        return Success;

        static string YesNo(bool value) => value ? "yes" : "no";

        static string MinGap(FiveBarReport r)
        {
            var inRange = r.InRange;
            if (inRange.Count < 2)
            {
                return "-";
            }

            var gap = Enumerable.Range(1, inRange.Count - 1).Min(i => inRange[i].Pose.Servo - inRange[i - 1].Pose.Servo);
            return Deg(gap).ToString("F1", CultureInfo.InvariantCulture);
        }
    }

    private void WriteChecks(FiveBarReport report)
    {
        var s = report.Settings;
        var inRange = report.InRange;
        stdout.WriteLine("checks (brainstorm-multistable-servo.md, Spec):");
        stdout.WriteLine($"  1 count       {Verdict(report.CountOk)}  {inRange.Count} minima in range, target {s.TargetCount}");
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  2 range       {Verdict(report.RangeOk)}  {report.Minima.Count - inRange.Count} minima out of range; of {report.Starts} starts, {report.LeftRange} left the range, {report.Saddles} ended at a saddle, {report.Unconverged} did not converge"));
        var gaps = Enumerable.Range(1, Math.Max(0, inRange.Count - 1)).Select(i => Deg(inRange[i].Pose.Servo - inRange[i - 1].Pose.Servo)).ToArray();
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  3 gaps        {Verdict(report.GapsOk)}  {(gaps.Length == 0 ? "no neighbours" : string.Join(", ", gaps.Select(g => g.ToString("F1", CultureInfo.InvariantCulture))) + " deg")}, minimum {Deg(s.MinimumGap):G3}"));
        stdout.WriteLine($"  4 transitions {Verdict(report.TransitionsOk)}  {report.Transitions.Count(t => t.Arrived)} of {report.Transitions.Count} arrive");
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  5 torque      {Verdict(report.TorqueOk)}  peak {report.PeakTorque:F3} N m, limit {s.TorqueLimit:G3}"));
        stdout.WriteLine($"  verdict       {(report.Passes ? "PASS" : "FAIL")}");

        static string Verdict(bool ok) => ok ? "pass" : "FAIL";
    }

    private void WriteBasinMap(FiveBarReport report)
    {
        var m = report.Map;
        stdout.WriteLine();
        stdout.WriteLine("where each start ends: rows servo angle, columns passive angle from -180 deg; A, B, ... the minimum (lower case if the start left the servo range), . no assembly, ? no minimum.");
        foreach (var side in new[] { 1, -1 })
        {
            stdout.WriteLine();
            stdout.WriteLine($"E {(side > 0 ? "left" : "right")} of C->D:");
            for (var i = 0; i < m.Servo.Count; i++)
            {
                var row = new StringBuilder();
                for (var j = 0; j < m.Passive.Count; j++)
                {
                    var label = m.Label(side, i, j);
                    row.Append(label switch
                    {
                        FiveBarBasinMap.NoAssembly => '.',
                        FiveBarBasinMap.NoMinimum => '?',
                        _ => m.Left(side, i, j) ? char.ToLowerInvariant(Letter(label)) : Letter(label),
                    });
                }

                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {Deg(m.Servo[i]),6:F1} | {row}"));
            }
        }
    }

    private static string Describe(FiveBarGeometry g)
    {
        string T(string name, FiveBarTendon t) =>
            string.Create(CultureInfo.InvariantCulture, $"tendon {name} anchor ({t.Anchor.X:G4}, {t.Anchor.Y:G4}) L0 {t.RestLength:G4}");
        return string.Create(CultureInfo.InvariantCulture,
            $"Base {g.BaseWidth:G4} m; links A-C {g.ServoLength:G4}, C-E {g.CouplerCLength:G4}, D-E {g.CouplerDLength:G4}, B-D {g.PassiveLength:G4} m, {g.LinkMass:G4} kg each; EA {g.AxialStiffness:G4} N; g {g.Gravity:G4} m/s2; {T("C", g.TendonC)}, {T("D", g.TendonD)}, {T("E", g.TendonE)}, {T("CE", g.TendonCE)} at {g.TendonCEPosition:G4} of C-E m.");
    }

    private static char Letter(int index) => index < 26 ? (char)('A' + index) : '+';

    private static string Label(int index, bool inRange) => inRange ? Letter(index).ToString() : $"{Letter(index)} (out)";

    private static double Deg(double radians) => radians * 180 / Math.PI;
}
