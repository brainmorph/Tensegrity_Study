using System.Globalization;
using System.Text;

namespace Tensegrity.Regression.Harness;

/// <summary>
/// Compares a scenario's output against a golden baseline. Strict by design:
/// missing or extra quantities, length changes, missing or stale tolerances,
/// and out-of-tolerance values are all failures.
/// </summary>
public static class BaselineComparer
{
    /// <summary>Out-of-tolerance values reported per quantity before truncating.</summary>
    public const int MaxReportedValuesPerQuantity = 5;

    public static ComparisonReport Compare(GoldenBaseline baseline, IReadOnlyDictionary<string, double[]> actual)
    {
        var failures = new List<string>();

        foreach (var name in baseline.Tolerances.Keys.Where(k => !baseline.Quantities.ContainsKey(k)).Order(StringComparer.Ordinal))
        {
            failures.Add($"tolerance '{name}' has no matching quantity in the baseline (stale tolerance).");
        }

        foreach (var name in actual.Keys.Where(k => !baseline.Quantities.ContainsKey(k)).Order(StringComparer.Ordinal))
        {
            failures.Add($"quantity '{name}' is produced but not in the baseline.");
        }

        foreach (var (name, expected) in baseline.Quantities.OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            if (!baseline.Tolerances.TryGetValue(name, out var tolerance))
            {
                failures.Add($"quantity '{name}' has no tolerance; every quantity needs an explicit one.");
                continue;
            }

            if (!actual.TryGetValue(name, out var values))
            {
                failures.Add($"quantity '{name}' is in the baseline but was not produced.");
                continue;
            }

            if (values.Length != expected.Length)
            {
                failures.Add($"quantity '{name}' has length {values.Length}, baseline has {expected.Length}.");
                continue;
            }

            CompareValues(name, expected, values, tolerance, failures);
        }

        return new ComparisonReport(baseline.Name, failures);
    }

    private static void CompareValues(string name, double[] expected, double[] actual, Tolerance tolerance, List<string> failures)
    {
        var bad = 0;
        for (var i = 0; i < expected.Length; i++)
        {
            if (tolerance.Accepts(expected[i], actual[i]))
            {
                continue;
            }

            bad++;
            if (bad <= MaxReportedValuesPerQuantity)
            {
                failures.Add(string.Create(
                    CultureInfo.InvariantCulture,
                    $"quantity '{name}'[{i}]: expected {expected[i]:R}, actual {actual[i]:R}, " +
                    $"diff {Math.Abs(actual[i] - expected[i]):G6}, allowed {tolerance.AllowedDifference(expected[i]):G6} ({tolerance})."));
            }
        }

        if (bad > MaxReportedValuesPerQuantity)
        {
            failures.Add($"quantity '{name}': {bad - MaxReportedValuesPerQuantity} more values out of tolerance (of {expected.Length}).");
        }
    }
}

/// <summary>Outcome of a baseline comparison.</summary>
public sealed class ComparisonReport(string baselineName, IReadOnlyList<string> failures)
{
    public string BaselineName { get; } = baselineName;

    public IReadOnlyList<string> Failures { get; } = failures;

    public bool Passed => Failures.Count == 0;

    public override string ToString()
    {
        if (Passed)
        {
            return $"Baseline '{BaselineName}': match.";
        }

        var text = new StringBuilder();
        text.AppendLine($"Baseline '{BaselineName}': {Failures.Count} failure(s). A regression failure is a stop signal: investigate before changing anything (PLAN.md 7a).");
        foreach (var failure in Failures)
        {
            text.Append("  - ").AppendLine(failure);
        }

        return text.ToString();
    }
}
