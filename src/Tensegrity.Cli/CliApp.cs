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
          run --topology FILE [--seed N] [--out DIR] [--dt S] [--duration S] [--frame-stride N]
              Simulate a topology file and write a run directory (manifest.json,
              frames.bin). dt, duration and frame stride default to the file's
              "simulation" block. --out defaults to runs/<date>_<name>_seed<N>.
              Exit code 3 means the run was written but a strut went into tension.
          dt-study --topology FILE [--dt S] [--duration S] [--levels N]
              Run at dt, dt/2, dt/4, ... and print how much each halving changes
              the trajectory (test 14). --dt is the coarsest step (default 4x the
              file's dt); --levels defaults to 5.
          export-csv RUN_DIR [--out FILE]
              Write a run's frames as CSV, one row per frame (default
              RUN_DIR/frames.csv).
          export-topologies [--out DIR]
              Write the reference topologies as JSON files (default topologies/).
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
        options.Allow("topology", "seed", "out", "dt", "duration", "frame-stride");
        var topologyPath = options.Required("topology");

        // Check every option before touching any file.
        var dtOption = options.Double("dt");
        var durationOption = options.Double("duration");
        var strideOption = options.Int("frame-stride");
        var seed = options.Long("seed") ?? 0;

        var model = TopologyFiles.Load(topologyPath);
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
        options.Allow("topology", "dt", "duration", "levels");
        var topologyPath = options.Required("topology");
        var dtOption = options.Double("dt");
        var durationOption = options.Double("duration");
        var levels = options.Int("levels") ?? 5;

        var model = TopologyFiles.Load(topologyPath);
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

    private sealed class UsageException(string message) : Exception(message);

    /// <summary><c>--name value</c> options and positional arguments.</summary>
    private sealed class Options
    {
        private readonly Dictionary<string, string> values = new(StringComparer.Ordinal);

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

                if (!options.values.TryAdd(name, e.Current))
                {
                    throw new UsageException($"option --{name} given twice.");
                }
            }

            return options;
        }

        public void Allow(params string[] names)
        {
            var unknown = values.Keys.FirstOrDefault(k => !names.Contains(k));
            if (unknown is not null)
            {
                throw new UsageException($"unknown option --{unknown}.");
            }
        }

        public string? Value(string name) => values.GetValueOrDefault(name);

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
