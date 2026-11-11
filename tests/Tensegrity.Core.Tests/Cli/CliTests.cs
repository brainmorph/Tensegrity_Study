using System.Globalization;
using Tensegrity.Cli;
using Tensegrity.RunFormat;
using Tensegrity.Testing;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Cli;

/// <summary>The command line, run in process against temporary directories.</summary>
public sealed class CliTests : IDisposable
{
    private readonly string root = Directory.CreateTempSubdirectory("tensegrity-cli-").FullName;
    private readonly StringWriter stdout = new();
    private readonly StringWriter stderr = new();

    public void Dispose() => Directory.Delete(root, recursive: true);

    private static string Topology(string name) => Path.Combine(RepoPaths.Root, "topologies", name + ".json");

    private string Dir(string name) => Path.Combine(root, name);

    private int Cli(params string[] args) =>
        new CliApp(stdout, stderr, () => new DateTime(2026, 11, 9, 12, 0, 0, DateTimeKind.Utc), () => ("abc123", false)).Run(args);

    [Fact]
    public void RunWritesARunDirectoryFromATopologyFile()
    {
        var exit = Cli("run", "--topology", Topology("planar_x"), "--seed", "7", "--duration", "0.1", "--out", Dir("run"));

        Assert.Equal(CliApp.Success, exit);
        var run = RunReader.Open(Dir("run"));
        Assert.Equal("planar_x", run.Manifest.Topology.Name);
        Assert.Equal(7, run.Manifest.Seed);
        Assert.Equal("2026-11-09T12:00:00Z", run.Manifest.CreatedUtc);
        Assert.Equal("abc123", run.Manifest.SimVersion);
        Assert.False(run.Manifest.SimDirty);
        Assert.Equal(Topology("planar_x"), run.Manifest.TopologySource);
        Assert.Equal(1_000, run.Manifest.TotalSteps);
        Assert.Equal(11, run.FrameCount);
        Assert.Contains("11 frames", stdout.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void CommandLineOverridesTheFileSettings()
    {
        var exit = Cli("run", "--topology", Topology("pendulum"), "--dt", "0.001", "--duration", "0.5", "--frame-stride", "25", "--out", Dir("run"));

        Assert.Equal(CliApp.Success, exit);
        var manifest = RunReader.Open(Dir("run")).Manifest;
        Assert.Equal(0.001, manifest.Dt);
        Assert.Equal(500, manifest.TotalSteps);
        Assert.Equal(21, manifest.FrameCount);
    }

    [Fact]
    public void EffortPrintsTheHoldingTorqueAcrossTheSweep()
    {
        var exit = Cli("effort", "--topology", Topology("locking_arm"), "--cable", "elbow_tendon", "--from", "0.99", "--to", "1.01", "--steps", "3");

        Assert.Equal(CliApp.Success, exit);
        var lines = stdout.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);
        Assert.Contains(lines, l => l.StartsWith("| scale | rest length (m) | tension (N) | shoulder (N m) | elbow (N m) |", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.StartsWith("| 1.0000 | 0.135069 | 7.005 | 0.0000 | 0.0000 |", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.StartsWith("| 1.0100 | 0.136419 | 0.000 | 0.0000 | 0.2060 |", StringComparison.Ordinal));
    }

    [Fact]
    public void EffortRejectsABadRange()
    {
        Assert.Equal(CliApp.Usage, Cli("effort", "--topology", Topology("locking_arm"), "--cable", "elbow_tendon", "--from", "1.1", "--to", "1.0"));
        Assert.Equal(CliApp.Usage, Cli("effort", "--topology", Topology("locking_arm"), "--cable", "elbow_tendon", "--from", "-0.1", "--to", "1.0"));
        Assert.Equal(CliApp.Failure, Cli("effort", "--topology", Topology("locking_arm"), "--cable", "nope"));
    }

    [Fact]
    public void EffortNeedsExactlyOneOfCableAndBend()
    {
        Assert.Equal(CliApp.Usage, Cli("effort", "--topology", Topology("locking_arm")));
        Assert.Equal(CliApp.Usage, Cli("effort", "--topology", Topology("locking_arm"), "--cable", "elbow_tendon", "--bend", "elbow"));
    }

    [Fact]
    public void EffortBendPrintsTensionsAndTorquesAgainstJointAngle()
    {
        var exit = Cli("effort", "--topology", Topology("locking_arm"), "--bend", "elbow", "--from", "-2", "--to", "2", "--steps", "3");

        Assert.Equal(CliApp.Success, exit);
        var lines = stdout.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);
        Assert.Contains(lines, l => l.StartsWith("| bend (deg) | joint angle (deg) | shoulder_tendon (N) | elbow_tendon (N) | shoulder (N m) | elbow (N m) |", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.StartsWith("| 0.000 | 0.000 | ", StringComparison.Ordinal) && l.Contains("| 0.0000 | 0.0000 |", StringComparison.Ordinal));
        Assert.Equal(3, lines.Count(l => l.StartsWith("| -2.000 |", StringComparison.Ordinal) || l.StartsWith("| 0.000 |", StringComparison.Ordinal) || l.StartsWith("| 2.000 |", StringComparison.Ordinal)));
        Assert.Equal(CliApp.Failure, Cli("effort", "--topology", Topology("locking_arm"), "--bend", "wrist"));
    }

    /// <summary>The perturbed_bar topology with its knock removed, so flags can put it back.</summary>
    private string UnknockedBar()
    {
        var document = RunFiles.ReadTopology(Topology("perturbed_bar"));
        document.Loads = null;
        var path = Path.Combine(root, "unknocked_bar.json");
        File.WriteAllText(path, RunFiles.SerializeTopology(document));
        return path;
    }

    /// <summary>Same frames, channel for channel, with the flag's load standing in for the file's knock.</summary>
    private static void AssertSameFrames(string expected, string actual, string loadId, double tolerance)
    {
        var a = RunReader.Open(expected);
        var b = RunReader.Open(actual);
        Assert.Equal(a.Channels, b.Channels.Select(c => c.Replace($"load.{loadId}.", "load.knock.", StringComparison.Ordinal)));
        Assert.Equal(a.FrameCount, b.FrameCount);
        for (var f = 0; f < a.FrameCount; f++)
        {
            var fa = a.Frame(f);
            var fb = b.Frame(f);
            for (var c = 0; c < fa.Length; c++)
            {
                Assert.True(Math.Abs(fa[c] - fb[c]) <= tolerance, $"Frame {f}, {a.Channels[c]}: {fa[c]} vs {fb[c]}.");
            }
        }
    }

    [Fact]
    public void ForceFlagReproducesALoadFromTheFileExactly()
    {
        Assert.Equal(CliApp.Success, Cli("run", "--topology", Topology("perturbed_bar"), "--duration", "0.8", "--out", Dir("file")));

        var exit = Cli("run", "--topology", UnknockedBar(), "--duration", "0.8", "--out", Dir("flag"),
            "--force", "bar,0.5,0,0.5,0,0,0.5025,120,240,0.505,0,0");

        Assert.Equal(CliApp.Success, exit);
        var load = Assert.Single(RunReader.Open(Dir("flag")).Manifest.Topology.Loads!);
        Assert.Equal("cli_force_1", load.Id);
        AssertSameFrames(Dir("file"), Dir("flag"), "cli_force_1", 0);
    }

    [Fact]
    public void ImpulseFlagIsATriangularPulse()
    {
        Assert.Equal(CliApp.Success, Cli("run", "--topology", Topology("perturbed_bar"), "--duration", "0.8", "--out", Dir("file")));

        // The file's knock: peak (120, 240) N over 5 ms is (0.3, 0.6) N·s.
        var exit = Cli("run", "--topology", UnknockedBar(), "--duration", "0.8", "--out", Dir("flag"),
            "--impulse", "bar,0.5,0,0.3,0.6,0.5,0.005");

        Assert.Equal(CliApp.Success, exit);
        var load = Assert.Single(RunReader.Open(Dir("flag")).Manifest.Topology.Loads!);
        Assert.Equal("cli_impulse_1", load.Id);
        Assert.Equal([0.5, 0.5025, 0.505], load.Schedule.Select(p => p.Time).ToArray(), (x, y) => Math.Abs(x - y) < 1e-15);
        Assert.Equal(120, load.Schedule[1].Fx, 12);
        Assert.Equal(240, load.Schedule[1].Fy, 12);
        AssertSameFrames(Dir("file"), Dir("flag"), "cli_impulse_1", 1e-9);
    }

    [Fact]
    public void PerturbationFlagsAddToTheFileAndRepeat()
    {
        var exit = Cli("run", "--topology", Topology("perturbed_bar"), "--duration", "0.01", "--out", Dir("run"),
            "--force", "bar,0,0,0,0,-1", "--force", "bar,-0.5,0,0,1,0,1,0,0", "--impulse", "bar,0.5,0,0,1,0,0.01");

        Assert.Equal(CliApp.Success, exit);
        var ids = RunReader.Open(Dir("run")).Manifest.Topology.Loads!.Select(l => l.Id);
        Assert.Equal(["knock", "cli_impulse_1", "cli_force_1", "cli_force_2"], ids);
    }

    [Fact]
    public void GroundDropFlagLowersTheGroundLine()
    {
        var exit = Cli("run", "--topology", Topology("ground_drop"), "--duration", "0.01", "--out", Dir("run"), "--ground-drop", "0.2,0.1,0.05");

        Assert.Equal(CliApp.Success, exit);
        var ground = RunReader.Open(Dir("run")).Manifest.Topology.Ground!;
        Assert.Equal([0.2, 0.2 + 0.1], ground.HeightSchedule!.Select(p => p.Time));
        Assert.Equal([ground.Height, ground.Height - 0.05], ground.HeightSchedule!.Select(p => p.Height));
    }

    [Theory]
    [InlineData("--impulse", "wing,0,0,1,0,0,0.01")] // No such link.
    [InlineData("--impulse", "bar,0,0,1,0,0")] // Missing duration.
    [InlineData("--impulse", "bar,0,0,1,0,0,0")] // Zero duration.
    [InlineData("--force", "bar,0,0,0,1")] // Incomplete breakpoint.
    [InlineData("--force", "bar,0,0,1,0,0,1,0,0")] // Times not increasing.
    [InlineData("--force", "bar,0,zero,0,0,1")] // Not a number.
    [InlineData("--ground-drop", "0,0.1,0.05")] // perturbed_bar has no ground.
    [InlineData("--dt", "0.001")] // Not repeatable (given twice below).
    public void BadPerturbationFlagsAreUsageErrors(string flag, string value)
    {
        var exit = Cli("run", "--topology", Topology("perturbed_bar"), "--dt", "0.001", "--out", Dir("run"), flag, value);

        Assert.Equal(CliApp.Usage, exit);
        Assert.False(Directory.Exists(Dir("run")));
    }

    [Fact]
    public void RunRefusesToOverwriteAnExistingRun()
    {
        Assert.Equal(CliApp.Success, Cli("run", "--topology", Topology("pendulum"), "--duration", "0.01", "--out", Dir("run")));

        Assert.Equal(CliApp.Failure, Cli("run", "--topology", Topology("pendulum"), "--duration", "0.01", "--out", Dir("run")));
        Assert.Contains("already exists", stderr.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void StrutInTensionIsAConfigurationErrorButTheRunIsKept()
    {
        // The pendulum bar hangs in tension; marking it a strut must be reported.
        var document = TopologyFiles.ToDocument(ReferenceTopologies.Pendulum());
        document.Links[0].Role = LinkRoles.Strut;
        var path = Path.Combine(root, "hanging_strut.json");
        File.WriteAllText(path, RunFiles.SerializeTopology(document));

        var exit = Cli("run", "--topology", path, "--duration", "0.01", "--out", Dir("run"));

        Assert.Equal(CliApp.StrutInTension, exit);
        Assert.Contains("struts in tension: bar", stderr.ToString(), StringComparison.Ordinal);
        Assert.True(File.Exists(Path.Combine(Dir("run"), RunFiles.Manifest)));
    }

    [Fact]
    public void ExportCsvWritesOneRowPerFrame()
    {
        Cli("run", "--topology", Topology("cable_oscillator"), "--duration", "0.05", "--frame-stride", "50", "--out", Dir("run"));
        var csv = Path.Combine(root, "frames.csv");

        Assert.Equal(CliApp.Success, Cli("export-csv", Dir("run"), "--out", csv));

        var run = RunReader.Open(Dir("run"));
        var lines = File.ReadAllLines(csv);
        Assert.Equal(string.Join(",", run.Channels), lines[0]);
        Assert.Equal(run.FrameCount + 1, lines.Length);
        var last = lines[^1].Split(',').Select(v => float.Parse(v, CultureInfo.InvariantCulture)).ToArray();
        Assert.Equal(run.Frame(run.FrameCount - 1).ToArray(), last);
    }

    [Fact]
    public void ExportTopologiesReproducesTheCommittedFiles()
    {
        Assert.Equal(CliApp.Success, Cli("export-topologies", "--out", Dir("topologies")));

        foreach (var name in ReferenceTopologies.Names)
        {
            Assert.Equal(File.ReadAllText(Topology(name)), File.ReadAllText(Path.Combine(Dir("topologies"), name + ".json")));
        }
    }

    [Fact]
    public void DtStudyPrintsAConvergenceTable()
    {
        var exit = Cli("dt-study", "--topology", Topology("pendulum"), "--dt", "0.004", "--duration", "0.4", "--levels", "3");

        Assert.Equal(CliApp.Success, exit);
        var output = stdout.ToString();
        Assert.Contains("| dt (s) | max change to dt/2 | ratio |", output, StringComparison.Ordinal);
        Assert.Contains("| 0.004 |", output, StringComparison.Ordinal);
        Assert.Contains("| 0.001 |", output, StringComparison.Ordinal);
    }

    [Theory]
    [InlineData]
    [InlineData("frobnicate")]
    [InlineData("run")]
    [InlineData("run", "--topology")]
    [InlineData("run", "--topology", "x.json", "--colour", "red")]
    [InlineData("run", "--topology", "x.json", "--dt", "fast")]
    [InlineData("export-csv")]
    public void BadUsageExitsWithUsageCode(params string[] args)
    {
        Assert.Equal(CliApp.Usage, Cli(args));
        Assert.Contains("usage: tensegrity", args.Length == 0 ? stdout.ToString() : stderr.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void MissingTopologyFileIsAnError()
    {
        Assert.Equal(CliApp.Failure, Cli("run", "--topology", Path.Combine(root, "nope.json"), "--out", Dir("run")));
    }
}
