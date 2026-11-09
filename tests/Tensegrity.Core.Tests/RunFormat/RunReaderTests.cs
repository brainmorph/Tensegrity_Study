using System.Buffers.Binary;
using System.Text.Json;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Tests.RunFormat;

/// <summary>The run format on its own: JSON shape, frame layout and reader validation.</summary>
public sealed class RunReaderTests : IDisposable
{
    private readonly string directory = Directory.CreateTempSubdirectory("tensegrity-runformat-").FullName;

    public void Dispose() => Directory.Delete(directory, recursive: true);

    private void WriteRun(RunManifest manifest, float[] values)
    {
        File.WriteAllText(Path.Combine(directory, RunFiles.Manifest), JsonSerializer.Serialize(manifest, RunFiles.Json));
        var bytes = new byte[4 * values.Length];
        for (var i = 0; i < values.Length; i++)
        {
            BinaryPrimitives.WriteSingleLittleEndian(bytes.AsSpan(4 * i), values[i]);
        }

        File.WriteAllBytes(Path.Combine(directory, RunFiles.Frames), bytes);
    }

    private static RunManifest TwoChannelManifest(int frames) => new()
    {
        FrameCount = frames,
        FrameLayout = new FrameLayout { Channels = [FrameChannels.Time, FrameChannels.LinkX("bar")] },
    };

    [Fact]
    public void ReadsFramesAndChannelsByName()
    {
        WriteRun(TwoChannelManifest(3), [0f, 1.5f, 0.1f, 2.5f, 0.2f, -3.25f]);

        var run = RunReader.Open(directory);

        Assert.Equal(3, run.FrameCount);
        Assert.Equal(2, run.ChannelCount);
        Assert.Equal([0.1f, 2.5f], run.Frame(1).ToArray());
        Assert.Equal(-3.25f, run.Value(2, "link.bar.x"));
        Assert.Equal([1.5f, 2.5f, -3.25f], run.Channel("link.bar.x"));
        Assert.True(run.HasChannel("time"));
        Assert.Throws<KeyNotFoundException>(() => run.ChannelIndex("link.nope.x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => run.Frame(3));
    }

    [Fact]
    public void RejectsFramesThatDoNotMatchTheManifest()
    {
        WriteRun(TwoChannelManifest(3), [0f, 1f, 2f, 3f, 4f]);

        Assert.Throws<InvalidDataException>(() => RunReader.Open(directory));
    }

    [Fact]
    public void RejectsUnknownFormatVersion()
    {
        var manifest = TwoChannelManifest(0);
        manifest.FormatVersion = 99;
        WriteRun(manifest, []);

        Assert.Throws<InvalidDataException>(() => RunReader.Open(directory));
    }

    [Fact]
    public void ManifestUsesSnakeCaseAndDocumentsTheFrameSize()
    {
        var json = JsonSerializer.Serialize(TwoChannelManifest(1), RunFiles.Json);

        Assert.Contains("\"format_version\": 1", json, StringComparison.Ordinal);
        Assert.Contains("\"frame_layout\"", json, StringComparison.Ordinal);
        Assert.Contains("\"frame_bytes\": 8", json, StringComparison.Ordinal);
        Assert.Contains("\"byte_order\": \"little\"", json, StringComparison.Ordinal);
    }

    [Fact]
    public void TopologyRoundTripsExactlyAndAcceptsComments()
    {
        var topology = new TopologyDocument
        {
            Name = "t",
            Links = [new LinkDocument { Id = "a", Mass = 0.1, Inertia = 1.0 / 3.0, Length = Math.PI, Pose = new PoseDocument { Theta = -Math.PI / 2 } }],
            Pins = [new PinDocument { Id = "p", A = AttachmentDocument.OnLink("a", -0.5, 0), B = AttachmentDocument.AtGround(0, 0) }],
            Simulation = new SimulationDocument { Dt = 1e-4, Duration = 2, FrameStride = 100 },
        };
        var text = RunFiles.SerializeTopology(topology);
        var path = Path.Combine(directory, "t.json");
        File.WriteAllText(path, "// annotated by hand\n" + text);

        var back = RunFiles.ReadTopology(path);

        Assert.Equal(text, RunFiles.SerializeTopology(back));
        Assert.Equal(1.0 / 3.0, back.Links[0].Inertia);
        Assert.Equal(-Math.PI / 2, back.Links[0].Pose.Theta);
        Assert.True(back.Pins[0].B.IsGround);
        Assert.DoesNotContain("is_ground", text, StringComparison.Ordinal);
        Assert.Contains("\"rest_length\"", RunFiles.SerializeTopology(new TopologyDocument { Cables = [new CableDocument()] }), StringComparison.Ordinal);
    }
}
