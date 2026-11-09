using System.Buffers.Binary;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Tensegrity.RunFormat;

/// <summary>File names and JSON settings shared by the writer and readers.</summary>
public static class RunFiles
{
    public const string Manifest = "manifest.json";
    public const string Frames = "frames.bin";

    /// <summary>
    /// snake_case names, indented, null members omitted. Doubles are written
    /// in shortest round-trip form, so values survive a write and read
    /// exactly. Comments and trailing commas are accepted on read, so
    /// hand-authored topology files can be annotated.
    /// </summary>
    public static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
    };

    public static TopologyDocument ReadTopology(string path)
    {
        var document = JsonSerializer.Deserialize<TopologyDocument>(File.ReadAllText(path), Json)
            ?? throw new InvalidDataException($"{path} is empty.");
        if (document.FormatVersion != TopologyDocument.CurrentFormatVersion)
        {
            throw new InvalidDataException(
                $"{path} has topology format version {document.FormatVersion}; this reader supports {TopologyDocument.CurrentFormatVersion}.");
        }

        return document;
    }

    /// <summary>The exact text a topology is saved as, with a trailing newline.</summary>
    public static string SerializeTopology(TopologyDocument document) =>
        JsonSerializer.Serialize(document, Json) + "\n";
}

/// <summary>
/// Read-only access to a run directory: the manifest and the float32
/// frames. Loads <c>frames.bin</c> into memory on open; runs at the
/// study's scale are a few megabytes.
/// </summary>
public sealed class RunReader
{
    private readonly float[] values;
    private readonly Dictionary<string, int> channelIndex;

    private RunReader(RunManifest manifest, float[] values)
    {
        Manifest = manifest;
        this.values = values;
        channelIndex = new Dictionary<string, int>(StringComparer.Ordinal);
        for (var i = 0; i < manifest.FrameLayout.Channels.Count; i++)
        {
            if (!channelIndex.TryAdd(manifest.FrameLayout.Channels[i], i))
            {
                throw new InvalidDataException($"Duplicate frame channel '{manifest.FrameLayout.Channels[i]}'.");
            }
        }
    }

    public RunManifest Manifest { get; }

    public int FrameCount => Manifest.FrameCount;

    public int ChannelCount => Manifest.FrameLayout.Channels.Count;

    public IReadOnlyList<string> Channels => Manifest.FrameLayout.Channels;

    public static RunReader Open(string directory)
    {
        var manifestPath = Path.Combine(directory, RunFiles.Manifest);
        var manifest = JsonSerializer.Deserialize<RunManifest>(File.ReadAllText(manifestPath), RunFiles.Json)
            ?? throw new InvalidDataException($"{manifestPath} is empty.");
        if (manifest.FormatVersion != RunManifest.CurrentFormatVersion)
        {
            throw new InvalidDataException(
                $"{manifestPath} has run format version {manifest.FormatVersion}; this reader supports {RunManifest.CurrentFormatVersion}.");
        }

        var layout = manifest.FrameLayout;
        if (layout.ByteOrder != "little" || layout.Scalar != "float32")
        {
            throw new InvalidDataException($"Unsupported frame layout {layout.ByteOrder}/{layout.Scalar}.");
        }

        var bytes = File.ReadAllBytes(Path.Combine(directory, RunFiles.Frames));
        var expected = (long)manifest.FrameCount * layout.FrameBytes;
        if (bytes.LongLength != expected)
        {
            throw new InvalidDataException(
                $"{RunFiles.Frames} has {bytes.LongLength} bytes; the manifest describes {manifest.FrameCount} frames of {layout.FrameBytes} bytes ({expected}).");
        }

        var values = new float[bytes.Length / 4];
        for (var i = 0; i < values.Length; i++)
        {
            values[i] = BinaryPrimitives.ReadSingleLittleEndian(bytes.AsSpan(4 * i, 4));
        }

        return new RunReader(manifest, values);
    }

    public bool HasChannel(string name) => channelIndex.ContainsKey(name);

    public int ChannelIndex(string name) =>
        channelIndex.TryGetValue(name, out var index)
            ? index
            : throw new KeyNotFoundException($"The run has no frame channel '{name}'.");

    /// <summary>All channel values of one frame, in channel order.</summary>
    public ReadOnlySpan<float> Frame(int frame)
    {
        if ((uint)frame >= (uint)FrameCount)
        {
            throw new ArgumentOutOfRangeException(nameof(frame), frame, $"The run has {FrameCount} frames.");
        }

        return values.AsSpan(frame * ChannelCount, ChannelCount);
    }

    public float Value(int frame, string channel) => Frame(frame)[ChannelIndex(channel)];

    /// <summary>One channel across every frame.</summary>
    public float[] Channel(string name)
    {
        var index = ChannelIndex(name);
        var series = new float[FrameCount];
        for (var f = 0; f < FrameCount; f++)
        {
            series[f] = values[(f * ChannelCount) + index];
        }

        return series;
    }
}
