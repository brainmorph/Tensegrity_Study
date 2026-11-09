using Tensegrity.Regression.Harness;
using Tensegrity.Regression.Scenarios;

namespace Tensegrity.Regression;

public class SmokeRegressionTests
{
    [Fact]
    public void SmokeScenarioMatchesGoldenBaseline()
    {
        var baseline = GoldenBaseline.Load(SmokeScenario.Name);

        var report = BaselineComparer.Compare(baseline, SmokeScenario.Run());

        Assert.True(report.Passed, report.ToString());
    }

    [Fact]
    public void SmokeBaselineRejectsOutputPerturbedBeyondTolerance()
    {
        // Proves the on-disk baseline and tolerances actually catch a change,
        // rather than accepting anything.
        var baseline = GoldenBaseline.Load(SmokeScenario.Name);
        var output = SmokeScenario.Run().ToDictionary(kv => kv.Key, kv => (double[])kv.Value.Clone());
        var tolerance = baseline.Tolerances["sine_samples"];
        output["sine_samples"][3] += 10 * tolerance.AllowedDifference(output["sine_samples"][3]) + 1e-15;

        var report = BaselineComparer.Compare(baseline, output);

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("'sine_samples'[3]", StringComparison.Ordinal));
    }
}
