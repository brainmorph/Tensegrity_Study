using Tensegrity.Regression.Harness;
using Tensegrity.Regression.Scenarios;
using Tensegrity.Testing;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Regression;

/// <summary>
/// The regression suite: every reference scenario against its golden baseline.
/// Each milestone adds its scenarios to <see cref="Scenarios"/> (PLAN.md 7a).
/// </summary>
public class ReferenceScenarioTests
{
    private static readonly IReferenceScenario[] Scenarios =
    [
        new SmokeScenario(),

        // M1
        new FreeFallScenario(),
        new SpinningBodyScenario(),
        new PendulumScenario(),

        // M2
        new CableOscillatorScenario(),
        new PlanarXScenario(),
        new PlanarXFreeScenario(),

        // M5
        new LockingArmScenario(),
        new LockingArmCollapseScenario(),

        // M6
        new GroundDropScenario(),
        new PerturbedBarScenario(),
        new LockingWingScenario(),
        new LockingWingCollapseScenario(),
        new LockingWingPreloadedScenario(),

        // M7
        new CoupledWingScenario(),
        new CoupledWingSwitchScenario(),
    ];

    public static TheoryData<string> All() => new(Scenarios.Select(s => s.Name));

    [Theory]
    [MemberData(nameof(All))]
    public void ScenarioMatchesGoldenBaseline(string name) =>
        RegressionCheck.Verify(Scenarios.Single(s => s.Name == name));

    [Fact]
    public void EveryGoldenFolderHasAScenario()
    {
        // A baseline with no scenario would silently stop being checked.
        var folders = Directory.GetDirectories(RepoPaths.GoldenRoot).Select(Path.GetFileName);

        Assert.Equal(Scenarios.Select(s => s.Name).Order(), folders.Order());
    }

    [Fact]
    public void SmokeBaselineRejectsOutputPerturbedBeyondTolerance()
    {
        // Proves the on-disk baseline and tolerances actually catch a change,
        // rather than accepting anything.
        var baseline = GoldenBaseline.Load("smoke");
        var output = new SmokeScenario().Run().ToDictionary(kv => kv.Key, kv => (double[])kv.Value.Clone());
        var tolerance = baseline.Tolerances["sine_samples"];
        output["sine_samples"][3] += (10 * tolerance.AllowedDifference(output["sine_samples"][3])) + 1e-15;

        var report = BaselineComparer.Compare(baseline, output);

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("'sine_samples'[3]", StringComparison.Ordinal));
    }
}
