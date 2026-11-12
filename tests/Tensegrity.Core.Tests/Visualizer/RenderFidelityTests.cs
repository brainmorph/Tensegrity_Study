using System.Globalization;
using Tensegrity.Cli;
using Tensegrity.RunFormat;
using Tensegrity.Testing;

namespace Tensegrity.Core.Tests.Visualizer;

/// <summary>
/// PLAN.md section 7, test 16: the viewer, run headless on a recorded run,
/// holds exactly the recorded float32 values in the nodes it draws, for
/// every frame. Links: position and angle, unit scale, no skew. Cables:
/// both endpoints and the tension, with an identity node transform. Pins:
/// position. The scene's own transform is the y-flip and nothing else.
/// </summary>
public sealed class RenderFidelityTests : IDisposable
{
    private const string Prefix = "FIDELITY ";

    private readonly string root = Directory.CreateTempSubdirectory("tensegrity-fidelity-").FullName;

    public void Dispose() => Directory.Delete(root, recursive: true);

    [GodotTheory]
    [InlineData("planar_x")]
    [InlineData("cable_oscillator")]
    [InlineData("toggle_arm_snap")]
    public void ViewerShowsEveryRecordedFrameExactly(string topology)
    {
        var runDirectory = Record(topology);
        var run = RunReader.Open(runDirectory);
        var topologyDocument = run.Manifest.Topology;

        var (exit, stdout, stderr) = GodotViewer.RunHeadless("--run", runDirectory, "--fidelity-dump");

        Assert.True(exit == 0, $"Viewer exited with {exit}.\n{stderr}");
        var lines = stdout.Split('\n')
            .Where(l => l.StartsWith(Prefix, StringComparison.Ordinal))
            .Select(l => l[Prefix.Length..].TrimEnd('\r').Split(' '))
            .ToList();

        // Cable anchors on the ground are drawn as one fixed body node, at their topology points.
        var anchors = topologyDocument.Cables.SelectMany(c => new[] { c.A, c.B }).Where(a => a.IsGround).SelectMany(a => a.Ground!.Select(v => (float)v)).ToArray();
        var bodies = anchors.Length > 0 ? 1 : 0;

        var scene = Assert.Single(lines, l => l[0] == "scene");
        Assert.Equal(["1", "0", "0", "-1", "0", "0"], scene[1..7]);
        var elements = topologyDocument.Links.Count + topologyDocument.Cables.Count + topologyDocument.Pins.Count;
        Assert.Equal(elements + bodies, int.Parse(scene[7], CultureInfo.InvariantCulture));
        Assert.Equal(["end", run.FrameCount.ToString(CultureInfo.InvariantCulture)], lines[^1]);
        if (bodies == 1)
        {
            var body = Assert.Single(lines, l => l[0] == "body");
            Assert.Equal([0f, 0f, 0f, .. anchors], body[1..].Select(Parse).ToArray());
        }
        else
        {
            Assert.DoesNotContain(lines, l => l[0] == "body");
        }

        var shown = lines.Where(l => l[0] is "link" or "cable" or "pin")
            .ToDictionary(l => (l[0], int.Parse(l[1], CultureInfo.InvariantCulture), l[2]), l => l[3..].Select(Parse).ToArray());
        Assert.Equal(run.FrameCount * elements, shown.Count);

        for (var f = 0; f < run.FrameCount; f++)
        {
            foreach (var link in topologyDocument.Links)
            {
                Assert.Equal(
                    [run.Value(f, FrameChannels.LinkX(link.Id)), run.Value(f, FrameChannels.LinkY(link.Id)), run.Value(f, FrameChannels.LinkTheta(link.Id)), 1f, 1f, 0f],
                    shown[("link", f, link.Id)]);
            }

            foreach (var cable in topologyDocument.Cables)
            {
                Assert.Equal(
                    [
                        run.Value(f, FrameChannels.CableAx(cable.Id)), run.Value(f, FrameChannels.CableAy(cable.Id)),
                        run.Value(f, FrameChannels.CableBx(cable.Id)), run.Value(f, FrameChannels.CableBy(cable.Id)),
                        run.Value(f, FrameChannels.CableTension(cable.Id)), 0f, 0f, 0f,
                    ],
                    shown[("cable", f, cable.Id)]);
            }

            foreach (var pin in topologyDocument.Pins)
            {
                Assert.Equal(
                    [run.Value(f, FrameChannels.PinX(pin.Id)), run.Value(f, FrameChannels.PinY(pin.Id)), 0f],
                    shown[("pin", f, pin.Id)]);
            }
        }
    }

    [GodotTheory]
    [InlineData("--fidelity-dump")]
    [InlineData("--run")]
    [InlineData("--unknown")]
    public void ViewerRejectsBadArguments(string argument)
    {
        var (exit, _, stderr) = GodotViewer.RunHeadless(argument);

        Assert.Equal(2, exit);
        Assert.Contains("usage:", stderr, StringComparison.Ordinal);
    }

    [GodotTheory]
    [InlineData("missing")]
    public void ViewerFailsCleanlyOnAMissingRun(string name)
    {
        var (exit, _, stderr) = GodotViewer.RunHeadless("--run", Path.Combine(root, name), "--fidelity-dump");

        Assert.Equal(1, exit);
        Assert.Contains("Cannot open run", stderr, StringComparison.Ordinal);
    }

    private static float Parse(string value) => float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);

    private string Record(string topology)
    {
        var directory = Path.Combine(root, topology);
        var exit = new CliApp(TextWriter.Null, TextWriter.Null, () => new DateTime(2026, 11, 9, 12, 0, 0, DateTimeKind.Utc), () => ("test", false))
            .Run(["run", "--topology", Path.Combine(RepoPaths.Root, "topologies", topology + ".json"), "--out", directory]);
        Assert.Equal(CliApp.Success, exit);
        return directory;
    }
}
