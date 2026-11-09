namespace Tensegrity.Testing.Scenarios;

/// <summary>Collects named series of samples in insertion order.</summary>
public sealed class SampleRecorder
{
    private readonly Dictionary<string, List<double>> series = new(StringComparer.Ordinal);

    public void Add(string name, double value)
    {
        if (!series.TryGetValue(name, out var list))
        {
            list = [];
            series[name] = list;
        }

        list.Add(value);
    }

    public IReadOnlyDictionary<string, double[]> ToOutput() =>
        series.ToDictionary(kv => kv.Key, kv => kv.Value.ToArray(), StringComparer.Ordinal);
}
