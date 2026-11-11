using System.Diagnostics;
using System.Globalization;
using System.Text;
using Tensegrity.Core.Analysis;
using Tensegrity.Core.Output;
using Tensegrity.RunFormat;
using Tensegrity.Topology;

namespace Tensegrity.Cli;

/// <summary>
/// The <c>tensegrity</c> command line. The only place with file-system side
/// effects beyond run output (PLAN.md 5). Exit codes: 0 success, 1 error,
/// 2 bad usage, 3 run written but a strut went into tension (a
/// configuration error for the topology, PLAN.md 4.1).
/// </summary>
public sealed class CliApp
{
    public const int Success = 0;
    public const int Failure = 1;
    public const int Usage = 2;
    public const int StrutInTension = 3;

    public const string UsageText = """
        usage: tensegrity <command> [options]

        commands:
          run --topology FILE [--seed N] [--out DIR] [--dt S] [--duration S] [--frame-stride N] [PERTURBATIONS]
              Simulate a topology file and write a run directory (manifest.json,
              frames.bin). dt, duration and frame stride default to the file's
              "simulation" block. --out defaults to runs/<date>_<name>_seed<N>.
              Exit code 3 means the run was written but a strut went into tension.
          dt-study --topology FILE [--dt S] [--duration S] [--levels N] [PERTURBATIONS]
              Run at dt, dt/2, dt/4, ... and print how much each halving changes
              the trajectory (test 14). --dt is the coarsest step (default 4x the
              file's dt); --levels defaults to 5.
          effort --topology FILE (--cable ID | --bend PIN) [--pins A,B] [--from X] [--to X] [--steps N]
              Holding torque at the pins (default all) that keeps a pose still,
              in N steps (default 11). With --cable, the file's initial pose as
              the cable's rest length is scaled from --from to --to (default
              0.95 to 1.05). With --bend, the pose bent at that pin by --from
              to --to degrees (default -10 to 10), turning its outer side.
              Redundant pins get the torques with the smallest sum of squares.
          sweep --topology FILE --vary SPEC=VALUES [--vary SPEC=VALUES] [--dt S] [--duration S]
                [--collapse-angle DEG] [--pins A,B] [--csv FILE] [PERTURBATIONS]
              Run the topology over a grid of one or two parameters and report which
              configurations lock and the holding torque at the pins (default all)
              in the starting pose. SPEC is rest:CABLE (rest length as a multiple
              of the file's) or anchor:CABLE.a.x (also .a.y, .b.x, .b.y: an offset
              in m to that end's point, with the rest length scaled to keep the
              cable's stretch). VALUES is FROM:TO:N or a comma-separated list.
              A configuration collapses when a joint angle moves more than
              --collapse-angle (default 90) from its start during the run.
          export-csv RUN_DIR [--out FILE]
              Write a run's frames as CSV, one row per frame (default
              RUN_DIR/frames.csv).
          export-topologies [--out DIR]
              Write the reference topologies as JSON files (default topologies/).

        perturbations, added to the topology for this run only (forces in the
        world frame, points in the link's body frame; --impulse and --force
        may be repeated):
          --impulse LINK,X,Y,JX,JY,START,DURATION
              A triangular force pulse delivering (JX, JY) N s.
          --force LINK,X,Y,T,FX,FY[,T,FX,FY...]
              A force piecewise-linear through the (T, FX, FY) breakpoints,
              held before the first and after the last.
          --ground-drop START,DURATION,DEPTH
              The ground line falls DEPTH m over DURATION s from START.
        """;

    private readonly TextWriter stdout;
    private readonly TextWriter stderr;
    private readonly Func<DateTime> utcNow;
    private readonly Func<(string Version, bool? Dirty)> simVersion;

    public CliApp(TextWriter stdout, TextWriter stderr, Func<DateTime>? utcNow = null, Func<(string, bool?)>? simVersion = null)
    {
        this.stdout = stdout;
        this.stderr = stderr;
        this.utcNow = utcNow ?? (() => DateTime.UtcNow);
        this.simVersion = simVersion ?? GitVersion;
    }

    public int Run(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help" or "help")
        {
            stdout.WriteLine(UsageText);
            return args.Length == 0 ? Usage : Success;
        }

        try
        {
            var options = Options.Parse(args.Skip(1));
            return args[0] switch
            {
                "run" => RunCommand(options),
                "dt-study" => DtStudyCommand(options),
                "export-csv" => ExportCsvCommand(options),
                "export-topologies" => ExportTopologiesCommand(options),
                "effort" => EffortCommand(options),
                "sweep" => SweepCommand(options),
                _ => throw new UsageException($"unknown command '{args[0]}'."),
            };
        }
        catch (UsageException e)
        {
            stderr.WriteLine($"tensegrity: {e.Message}");
            stderr.WriteLine(UsageText);
            return Usage;
        }
        catch (Exception e) when (e is IOException or InvalidDataException or FormatException or ArgumentException or UnauthorizedAccessException or System.Text.Json.JsonException)
        {
            stderr.WriteLine($"tensegrity: {e.Message}");
            return Failure;
        }
    }

    private int RunCommand(Options options)
    {
        options.Allow(["topology", "seed", "out", "dt", "duration", "frame-stride", .. PerturbationFlags.Names]);
        var topologyPath = options.Required("topology");

        // Check every option before touching any file.
        var dtOption = options.Double("dt");
        var durationOption = options.Double("duration");
        var strideOption = options.Int("frame-stride");
        var seed = options.Long("seed") ?? 0;

        var model = TopologyFiles.Load(topologyPath, document => PerturbationFlags.Apply(document, options));
        var simulation = model.Simulation ?? new SimulationDocument();
        var dt = dtOption ?? simulation.Dt;
        var duration = durationOption ?? simulation.Duration;
        var stride = strideOption ?? simulation.FrameStride;
        if (dt <= 0 || duration <= 0)
        {
            throw new UsageException($"{topologyPath} has no simulation block; give --dt and --duration.");
        }

        var now = utcNow();
        var outDir = options.Value("out")
            ?? Path.Combine("runs", string.Create(CultureInfo.InvariantCulture, $"{now:yyyy-MM-dd}_{model.Name}_seed{seed}"));
        var (version, dirty) = simVersion();
        var settings = RunSettings.ForDuration(dt, duration, stride, simulation.EventTolerance);
        var metadata = new RunMetadata(
            model.Name,
            model.Description,
            seed,
            now.ToString("yyyy-MM-ddTHH:mm:ssZ", CultureInfo.InvariantCulture),
            version,
            dirty,
            topologyPath);

        var manifest = RunRecorder.Record(model.System, model.CreateInitialState(), settings, metadata, outDir);
        var d = manifest.Diagnostics;
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"wrote {outDir}: {manifest.FrameCount} frames, {manifest.TotalSteps} steps of {manifest.Dt:R} s"));
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  events {d.EventCount} (unresolved {d.UnresolvedEventSteps}), energy balance error {d.MaxEnergyBalanceError:G3} J, pin residual {d.MaxPinPositionResidual:G3} m"));
        if (d.StrutsInTension.Count > 0)
        {
            stderr.WriteLine($"tensegrity: configuration error: struts in tension: {string.Join(", ", d.StrutsInTension)} (see diagnostics.axial_forces in the manifest).");
            return StrutInTension;
        }

        return Success;
    }

    private int DtStudyCommand(Options options)
    {
        options.Allow(["topology", "dt", "duration", "levels", .. PerturbationFlags.Names]);
        var topologyPath = options.Required("topology");
        var dtOption = options.Double("dt");
        var durationOption = options.Double("duration");
        var levels = options.Int("levels") ?? 5;

        var model = TopologyFiles.Load(topologyPath, document => PerturbationFlags.Apply(document, options));
        var simulation = model.Simulation ?? new SimulationDocument();
        var coarsest = dtOption ?? (4 * simulation.Dt);
        var duration = durationOption ?? simulation.Duration;
        if (coarsest <= 0 || duration <= 0)
        {
            throw new UsageException($"{topologyPath} has no simulation block; give --dt and --duration.");
        }

        var study = StepSizeStudy.Run(model.System, model.InitialState, coarsest, duration, levels, simulation.EventTolerance);
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture, $"dt study: {model.Name}, {duration:R} s, sampled at {StepSizeStudy.Samples} times"));
        stdout.WriteLine();
        stdout.WriteLine("| dt (s) | max change to dt/2 | ratio | energy balance error (J) | events |");
        stdout.WriteLine("|---|---|---|---|---|");
        foreach (var level in study)
        {
            stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {level.Dt:G4} | {Format(level.ChangeToNext)} | {(level.Ratio is { } r ? r.ToString("F1", CultureInfo.InvariantCulture) : "-")} | {level.EnergyBalanceError:E2} | {level.Events} |"));
        }

        return Success;

        static string Format(double? value) => value is { } v ? v.ToString("E2", CultureInfo.InvariantCulture) : "-";
    }

    private int EffortCommand(Options options)
    {
        options.Allow("topology", "cable", "bend", "pins", "from", "to", "steps");
        var topologyPath = options.Required("topology");
        var cable = options.Value("cable");
        var bend = options.Value("bend");
        if ((cable is null) == (bend is null))
        {
            throw new UsageException("effort needs exactly one of --cable and --bend.");
        }

        var from = options.Double("from") ?? (cable is null ? -10 : 0.95);
        var to = options.Double("to") ?? (cable is null ? 10 : 1.05);
        var steps = options.Int("steps") ?? 11;
        if (steps < 2 || !(to > from) || (cable is not null && !(from > 0)))
        {
            throw new UsageException("effort needs --from < --to (and 0 < --from with --cable) and --steps of at least 2.");
        }

        var document = RunFiles.ReadTopology(topologyPath);
        var pins = options.Value("pins")?.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
            ?? document.Pins.Select(p => p.Id).ToArray();
        var values = Enumerable.Range(0, steps).Select(i => from + ((to - from) * i / (steps - 1))).ToArray();
        var pinColumns = string.Join(" | ", pins.Select(p => $"{p} (N m)"));
        var redundancy = 0;

        if (cable is not null)
        {
            var points = EffortSweep.Run(document, cable, pins, values);
            stdout.WriteLine($"effort: {document.Name}, cable {cable} scaled, holding torque in the initial pose");
            stdout.WriteLine();
            stdout.WriteLine($"| scale | rest length (m) | tension (N) | {pinColumns} | unheld accel |");
            stdout.WriteLine($"|---|---|---|{string.Concat(pins.Select(_ => "---|"))}---|");
            foreach (var point in points)
            {
                var unheld = Math.Max(point.MaxLinearAcceleration, point.MaxAngularAcceleration);
                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"| {point.Scale:F4} | {point.RestLength:F6} | {point.Tension:F3} | {Torques(point.Torques)} | {unheld:E1} |"));
                redundancy = Math.Max(redundancy, point.Redundancy);
            }
        }
        else
        {
            var points = EffortSweep.Bend(document, bend!, pins, values.Select(v => v * Math.PI / 180));
            var cableColumns = string.Join(" | ", document.Cables.Select(c => $"{c.Id} (N)"));
            stdout.WriteLine($"effort: {document.Name}, pin {bend} bent, holding torque in the bent pose");
            stdout.WriteLine();
            stdout.WriteLine($"| bend (deg) | joint angle (deg) | {cableColumns} | {pinColumns} | unheld accel |");
            stdout.WriteLine($"|---|---|{string.Concat(document.Cables.Select(_ => "---|"))}{string.Concat(pins.Select(_ => "---|"))}---|");
            foreach (var point in points)
            {
                var tensions = string.Join(" | ", point.Tensions.Select(t => t.ToString("F3", CultureInfo.InvariantCulture)));
                var unheld = Math.Max(point.MaxLinearAcceleration, point.MaxAngularAcceleration);
                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"| {point.Offset * 180 / Math.PI:F3} | {point.JointAngle * 180 / Math.PI:F3} | {tensions} | {Torques(point.Torques)} | {unheld:E1} |"));
                redundancy = Math.Max(redundancy, point.Redundancy);
            }
        }

        if (redundancy > 0)
        {
            stdout.WriteLine();
            stdout.WriteLine($"The pins are redundant ({redundancy} torque combinations move nothing): torques are the set with the smallest sum of squares.");
        }

        return Success;

        static string Torques(IEnumerable<double> torques) =>
            string.Join(" | ", torques.Select(t => (Math.Round(t, 4) + 0.0).ToString("F4", CultureInfo.InvariantCulture)));
    }

    private int SweepCommand(Options options)
    {
        options.Allow(["topology", "vary", "dt", "duration", "collapse-angle", "pins", "csv", .. PerturbationFlags.Names]);
        var topologyPath = options.Required("topology");
        var varies = options.All("vary");
        if (varies.Count is < 1 or > 2)
        {
            throw new UsageException("sweep needs one or two --vary options.");
        }

        var axes = varies.Select(ParseAxis).ToArray();
        var dtOption = options.Double("dt");
        var durationOption = options.Double("duration");
        var collapseDegrees = options.Double("collapse-angle") ?? 90;
        if (!(collapseDegrees > 0))
        {
            throw new UsageException("--collapse-angle must be positive.");
        }

        var document = RunFiles.ReadTopology(topologyPath);
        PerturbationFlags.Apply(document, options);
        var simulation = document.Simulation ?? new SimulationDocument();
        var dt = dtOption ?? simulation.Dt;
        var duration = durationOption ?? simulation.Duration;
        if (dt <= 0 || duration <= 0)
        {
            throw new UsageException($"{topologyPath} has no simulation block; give --dt and --duration.");
        }

        var pins = options.Value("pins")?.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
            ?? document.Pins.Select(p => p.Id).ToArray();
        var settings = new SweepSettings(dt, duration, collapseDegrees * Math.PI / 180, pins, simulation.EventTolerance);
        var points = ParameterSweep.Run(document, axes, settings);

        var axisColumns = string.Join(" | ", axes.Select(a => a.Parameter.Spec));
        var pinColumns = string.Concat(pins.Select(p => $" {p} (N m) |"));
        stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"sweep: {document.Name}, {points.Count} configurations, {duration:R} s each at dt {dt:R} s, collapse past {collapseDegrees:R} deg"));
        stdout.WriteLine();
        stdout.WriteLine($"| {axisColumns} | outcome | max change (deg) | final change (deg) | collapse (s) |{pinColumns} unheld accel |");
        stdout.WriteLine($"|{string.Concat(axes.Select(_ => "---|"))}---|---|---|---|{string.Concat(pins.Select(_ => "---|"))}---|");
        foreach (var point in points)
        {
            var values = string.Join(" | ", point.Values.Select(v => v.ToString("G6", CultureInfo.InvariantCulture)));
            var torques = string.Concat(point.Torques.Select(t => $" {(Math.Round(t, 4) + 0.0).ToString("F4", CultureInfo.InvariantCulture)} |"));
            stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {values} | {(point.Locked ? "locked" : $"collapsed ({point.CollapsedJoint})")} | {Degrees(point.MaxDeviation)} | {Degrees(point.FinalDeviation)} | {(point.CollapseTime is { } t ? t.ToString("F4", CultureInfo.InvariantCulture) : "-")} |{torques} {point.UnheldAcceleration:E1} |"));
        }

        stdout.WriteLine();
        stdout.WriteLine($"locked {points.Count(p => p.Locked)} of {points.Count}.");
        if (axes.Length == 2)
        {
            // A map of the grid: rows are the first axis, columns the second; L locked, x collapsed.
            stdout.WriteLine();
            stdout.WriteLine($"rows {axes[0].Parameter.Spec}, columns {axes[1].Parameter.Spec}; L locked, x collapsed.");
            stdout.WriteLine($"columns: {string.Join(", ", axes[1].Values.Select(v => v.ToString("G6", CultureInfo.InvariantCulture)))}");
            var columns = axes[1].Values.Count;
            for (var r = 0; r < axes[0].Values.Count; r++)
            {
                var row = string.Concat(points.Skip(r * columns).Take(columns).Select(p => p.Locked ? " L" : " x"));
                stdout.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {axes[0].Values[r],12:G6} |{row}"));
            }
        }
        else
        {
            for (var i = 1; i < points.Count; i++)
            {
                if (points[i].Locked != points[i - 1].Locked)
                {
                    stdout.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"{(points[i].Locked ? "locks" : "collapses")} between {points[i - 1].Values[0]:G6} and {points[i].Values[0]:G6}."));
                }
            }
        }

        if (points.Any(p => p.Redundancy > 0))
        {
            stdout.WriteLine();
            stdout.WriteLine("The pins are redundant: torques are the set with the smallest sum of squares.");
        }

        if (options.Value("csv") is { } csv)
        {
            WriteSweepCsv(csv, document, axes, pins, points);
            stdout.WriteLine($"wrote {csv}");
        }

        return Success;

        static string Degrees(double radians) =>
            double.IsNaN(radians) ? "-" : (radians * 180 / Math.PI).ToString("F3", CultureInfo.InvariantCulture);

        static SweepAxis ParseAxis(string vary)
        {
            var equals = vary.IndexOf('=', StringComparison.Ordinal);
            if (equals < 0)
            {
                throw new UsageException($"--vary '{vary}' needs SPEC=VALUES.");
            }

            try
            {
                return new SweepAxis(SweepParameter.Parse(vary[..equals]), ParameterSweep.ParseValues(vary[(equals + 1)..]));
            }
            catch (FormatException e)
            {
                throw new UsageException($"--vary '{vary}': {e.Message}");
            }
        }
    }

    private static void WriteSweepCsv(string path, TopologyDocument document, IReadOnlyList<SweepAxis> axes, IReadOnlyList<string> pins, IReadOnlyList<SweepPoint> points)
    {
        var text = new StringBuilder();
        IEnumerable<string> header =
        [
            .. axes.Select(a => a.Parameter.Spec),
            "locked", "max_change_rad", "final_change_rad", "collapse_time_s", "collapsed_joint",
            .. document.Cables.Select(c => $"tension.{c.Id}"),
            .. pins.Select(p => $"torque.{p}"),
            "unheld_acceleration",
        ];
        text.AppendLine(string.Join(",", header));
        foreach (var point in points)
        {
            IEnumerable<string> row =
            [
                .. point.Values.Select(Number),
                point.Locked ? "1" : "0", Number(point.MaxDeviation), Number(point.FinalDeviation), point.CollapseTime is { } t ? Number(t) : string.Empty, point.CollapsedJoint ?? string.Empty,
                .. point.Tensions.Select(Number),
                .. point.Torques.Select(Number),
                Number(point.UnheldAcceleration),
            ];
            text.AppendLine(string.Join(",", row));
        }

        File.WriteAllText(path, text.ToString());

        static string Number(double value) => double.IsNaN(value) ? string.Empty : value.ToString("R", CultureInfo.InvariantCulture);
    }

    private int ExportCsvCommand(Options options)
    {
        options.Allow("out");
        var runDir = options.Positional.Count == 1 ? options.Positional[0] : throw new UsageException("export-csv needs one run directory.");
        var run = RunReader.Open(runDir);
        var outPath = options.Value("out") ?? Path.Combine(runDir, "frames.csv");
        var text = new StringBuilder();
        text.AppendLine(string.Join(",", run.Channels));
        for (var f = 0; f < run.FrameCount; f++)
        {
            var frame = run.Frame(f);
            for (var c = 0; c < frame.Length; c++)
            {
                if (c > 0)
                {
                    text.Append(',');
                }

                text.Append(frame[c].ToString("R", CultureInfo.InvariantCulture));
            }

            text.AppendLine();
        }

        File.WriteAllText(outPath, text.ToString());
        stdout.WriteLine($"wrote {outPath}: {run.FrameCount} rows, {run.ChannelCount} columns");
        return Success;
    }

    private int ExportTopologiesCommand(Options options)
    {
        options.Allow("out");
        var outDir = options.Value("out") ?? "topologies";
        Directory.CreateDirectory(outDir);
        foreach (var name in ReferenceTopologies.Names)
        {
            var path = Path.Combine(outDir, name + ".json");
            TopologyFiles.Save(ReferenceTopologies.Create(name), path);
            stdout.WriteLine($"wrote {path}");
        }

        return Success;
    }

    /// <summary>The current git commit and whether the tree has changes, or "unknown" if git is unavailable.</summary>
    private static (string, bool?) GitVersion()
    {
        var sha = Git("rev-parse --short=12 HEAD");
        if (sha is null)
        {
            return ("unknown", null);
        }

        var status = Git("status --porcelain --untracked-files=no");
        return (sha, status is null ? null : status.Length > 0);
    }

    private static string? Git(string arguments)
    {
        try
        {
            using var process = Process.Start(new ProcessStartInfo("git", arguments)
            {
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
            });
            if (process is null)
            {
                return null;
            }

            var output = process.StandardOutput.ReadToEnd().Trim();
            process.WaitForExit();
            return process.ExitCode == 0 ? output : null;
        }
        catch (System.ComponentModel.Win32Exception)
        {
            return null;
        }
    }

    internal sealed class UsageException(string message) : Exception(message);

    /// <summary>
    /// <c>--name value</c> options and positional arguments. An option may be
    /// given more than once only if it is read with <see cref="All"/>.
    /// </summary>
    internal sealed class Options
    {
        /// <summary>Options that may be given more than once.</summary>
        private static readonly string[] Repeatable = ["impulse", "force", "vary"];

        private readonly Dictionary<string, List<string>> values = new(StringComparer.Ordinal);

        public List<string> Positional { get; } = [];

        public static Options Parse(IEnumerable<string> args)
        {
            var options = new Options();
            using var e = args.GetEnumerator();
            while (e.MoveNext())
            {
                var arg = e.Current;
                if (!arg.StartsWith("--", StringComparison.Ordinal))
                {
                    options.Positional.Add(arg);
                    continue;
                }

                var name = arg[2..];
                if (!e.MoveNext())
                {
                    throw new UsageException($"option --{name} needs a value.");
                }

                if (!options.values.TryGetValue(name, out var list))
                {
                    options.values[name] = list = [];
                }

                list.Add(e.Current);
            }

            return options;
        }

        /// <summary>Rejects options not in <paramref name="names"/>, and repeats of any but the repeatable ones.</summary>
        public void Allow(params string[] names)
        {
            var unknown = values.Keys.FirstOrDefault(k => !names.Contains(k));
            if (unknown is not null)
            {
                throw new UsageException($"unknown option --{unknown}.");
            }

            var repeated = values.FirstOrDefault(v => v.Value.Count > 1 && !Repeatable.Contains(v.Key)).Key;
            if (repeated is not null)
            {
                throw new UsageException($"option --{repeated} given twice.");
            }
        }

        /// <summary>Every value given for <paramref name="name"/>, in order.</summary>
        public IReadOnlyList<string> All(string name) => values.TryGetValue(name, out var list) ? list : [];

        public string? Value(string name) => values.TryGetValue(name, out var list) ? list[^1] : null;

        public string Required(string name) => Value(name) ?? throw new UsageException($"missing --{name}.");

        public double? Double(string name) => Parse(name, s => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture));

        public int? Int(string name) => Parse(name, s => int.Parse(s, NumberStyles.Integer, CultureInfo.InvariantCulture));

        public long? Long(string name) => Parse(name, s => long.Parse(s, NumberStyles.Integer, CultureInfo.InvariantCulture));

        private T? Parse<T>(string name, Func<string, T> parse)
            where T : struct
        {
            var value = Value(name);
            if (value is null)
            {
                return null;
            }

            try
            {
                return parse(value);
            }
            catch (FormatException)
            {
                throw new UsageException($"--{name} '{value}' is not a valid {typeof(T).Name}.");
            }
        }
    }
}
