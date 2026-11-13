using Tensegrity.Cli;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Cli;

/// <summary>The <c>minima</c> command, run in process.</summary>
public sealed class MinimaCommandTests : IDisposable
{
    private readonly string root = Directory.CreateTempSubdirectory("tensegrity-minima-").FullName;
    private readonly StringWriter stdout = new();
    private readonly StringWriter stderr = new();

    public void Dispose() => Directory.Delete(root, recursive: true);

    private int Cli(params string[] args) =>
        new CliApp(stdout, stderr, () => new DateTime(2026, 11, 13, 12, 0, 0, DateTimeKind.Utc), () => ("abc123", false)).Run(args);

    private string[] Lines() => stdout.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);

    [Fact]
    public void ReportsMinimaTransitionsAndChecks()
    {
        var exit = Cli("minima", "--grid", "15");

        Assert.Equal(CliApp.Success, exit);
        var lines = Lines();
        Assert.Contains(lines, l => l.StartsWith("| # | servo (deg) | passive (deg) |", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.StartsWith("| A | ", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.StartsWith("| from | to | arrived |", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.Contains("1 count", StringComparison.Ordinal));
        Assert.Contains(lines, l => l.Contains("verdict       FAIL", StringComparison.Ordinal));
    }

    [Fact]
    public void SavesAMinimumAsATopologyAndWritesThePaths()
    {
        var topology = Path.Combine(root, "five_bar.json");
        var csv = Path.Combine(root, "paths.csv");

        var exit = Cli("minima", "--grid", "15", "--set", "tendon_e.rest=0.24", "--save", topology, "--at", "1", "--csv", csv, "--map", "yes");

        Assert.Equal(CliApp.Success, exit);
        var model = TopologyFiles.Load(topology);
        Assert.Equal(4, model.System.BodyCount);
        Assert.Equal(5, model.System.Pins.Count);
        Assert.Equal(4, model.System.Cables.Count);
        Assert.True(model.System.PinPositionResidual(model.InitialState) < 1e-12);
        Assert.StartsWith("from,to,servo_rad,passive_rad,torque_n_m", File.ReadAllText(csv), StringComparison.Ordinal);
        Assert.Contains(Lines(), l => l.StartsWith("E left of C->D:", StringComparison.Ordinal));
    }

    [Fact]
    public void SweepsOneParameterAndMapsTwo()
    {
        Assert.Equal(CliApp.Success, Cli("minima", "--grid", "20", "--vary", "tendon_e.rest=0.20,0.24"));
        Assert.Contains(Lines(), l => l.StartsWith("| tendon_e.rest | in range |", StringComparison.Ordinal));
        Assert.Contains(Lines(), l => l.StartsWith("pass 0 of 2.", StringComparison.Ordinal));

        stdout.GetStringBuilder().Clear();
        Assert.Equal(CliApp.Success, Cli("minima", "--grid", "30", "--vary", "tendon_e.rest=0.20,0.24", "--vary", "tendon_c.rest=0.22,0.23"));
        Assert.Contains(Lines(), l => l.StartsWith("rows tendon_e.rest, columns tendon_c.rest", StringComparison.Ordinal));
    }

    [Fact]
    public void RejectsBadOptions()
    {
        Assert.Equal(CliApp.Usage, Cli("minima", "--set", "tendon_q.x=1"));
        Assert.Equal(CliApp.Usage, Cli("minima", "--vary", "tendon_e.rest"));
        Assert.Equal(CliApp.Usage, Cli("minima", "--grid", "0"));
        Assert.Equal(CliApp.Usage, Cli("minima", "--save", Path.Combine(root, "x.json")));
        Assert.Equal(CliApp.Usage, Cli("minima", "--vary", "tendon_e.rest=0.2,0.24", "--map", "yes"));
        Assert.Equal(CliApp.Usage, Cli("minima", "--grid", "30", "--save", Path.Combine(root, "x.json"), "--at", "99"));
        Assert.Equal(CliApp.Failure, Cli("minima", "--grid", "30", "--set", "tendon_c.rest=-1"));
    }
}
