using System.Text.Json;
using System.Text.Json.Serialization;
using Tensegrity.Testing;

namespace Tensegrity.Regression.Harness;

/// <summary>
/// A committed known-good result, stored in <c>tests/golden/&lt;name&gt;/</c> as:
/// <list type="bullet">
/// <item><c>baseline.json</c>: named quantities, each an array of doubles
/// (a scalar is an array of length one).</item>
/// <item><c>tolerances.json</c>: one explicit tolerance per quantity. There is
/// no default; a quantity without a tolerance fails comparison.</item>
/// </list>
/// Baselines change only deliberately (PLAN.md section 7a).
/// </summary>
public sealed class GoldenBaseline
{
    public const string BaselineFile = "baseline.json";
    public const string TolerancesFile = "tolerances.json";

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        ReadCommentHandling = JsonCommentHandling.Skip,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
    };

    public GoldenBaseline(
        string name,
        IReadOnlyDictionary<string, double[]> quantities,
        IReadOnlyDictionary<string, Tolerance> tolerances)
    {
        Name = name;
        Quantities = quantities;
        Tolerances = tolerances;
    }

    public string Name { get; }

    public IReadOnlyDictionary<string, double[]> Quantities { get; }

    public IReadOnlyDictionary<string, Tolerance> Tolerances { get; }

    /// <summary>Loads <c>tests/golden/&lt;name&gt;/</c> from the source tree.</summary>
    public static GoldenBaseline Load(string name) =>
        LoadFrom(Path.Combine(RepoPaths.GoldenRoot, name));

    public static GoldenBaseline LoadFrom(string directory)
    {
        var baseline = Read<BaselineDto>(Path.Combine(directory, BaselineFile));
        var tolerances = Read<Dictionary<string, ToleranceDto>>(Path.Combine(directory, TolerancesFile));

        return new GoldenBaseline(
            baseline.Name ?? throw new InvalidDataException($"{BaselineFile} in {directory} has no name."),
            baseline.Quantities ?? throw new InvalidDataException($"{BaselineFile} in {directory} has no quantities."),
            tolerances.ToDictionary(kv => kv.Key, kv => new Tolerance(kv.Value.Absolute, kv.Value.Relative)));
    }

    private static T Read<T>(string path)
    {
        using var stream = File.OpenRead(path);
        return JsonSerializer.Deserialize<T>(stream, JsonOptions)
            ?? throw new InvalidDataException($"{path} is empty.");
    }

    private sealed class BaselineDto
    {
        [JsonPropertyName("name")]
        public string? Name { get; set; }

        [JsonPropertyName("description")]
        public string? Description { get; set; }

        [JsonPropertyName("quantities")]
        public Dictionary<string, double[]>? Quantities { get; set; }
    }

    private sealed class ToleranceDto
    {
        [JsonPropertyName("absolute")]
        [JsonRequired]
        public double Absolute { get; set; }

        [JsonPropertyName("relative")]
        [JsonRequired]
        public double Relative { get; set; }
    }
}
