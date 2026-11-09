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
        Assert.Equal(CliApp.Failure, Cli("effort", "--topology", Topology("locking_arm"), "--cable", "nope"));
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
