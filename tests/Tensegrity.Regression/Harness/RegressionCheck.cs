using Tensegrity.Testing;

namespace Tensegrity.Regression.Harness;

/// <summary>
/// Runs a reference scenario and compares it to its golden baseline.
///
/// Deliberate baseline update (PLAN.md 7a): set
/// <c>TENSEGRITY_UPDATE_GOLDEN</c> to a comma-separated list of scenario
/// names, e.g. <c>TENSEGRITY_UPDATE_GOLDEN=pendulum dotnet test</c>. Each named
/// scenario's baseline.json is rewritten from current output and its test
/// FAILS on purpose, so an update can never pass silently. Review the diff,
/// explain it in the commit message, then rerun without the variable.
/// There is no "update all" option. tolerances.json is always written by
/// hand and must exist before a baseline can be written.
/// </summary>
public static class RegressionCheck
{
    public const string UpdateVariable = "TENSEGRITY_UPDATE_GOLDEN";

    public static void Verify(IReferenceScenario scenario)
    {
        var directory = Path.Combine(RepoPaths.GoldenRoot, scenario.Name);
        var output = scenario.Run();

        if (UpdateRequested(scenario.Name))
        {
            if (!File.Exists(Path.Combine(directory, GoldenBaseline.TolerancesFile)))
            {
                Assert.Fail($"Write {Path.Combine(directory, GoldenBaseline.TolerancesFile)} by hand before generating a baseline.");
            }

            GoldenWriter.Write(directory, scenario, output);
            Assert.Fail(
                $"Baseline '{scenario.Name}' was rewritten because {UpdateVariable} names it. " +
                "Review the diff in tests/golden/, record why it changed in the commit message, and rerun without the variable.");
        }

        if (!File.Exists(Path.Combine(directory, GoldenBaseline.BaselineFile)))
        {
            Assert.Fail($"No baseline for '{scenario.Name}'. Add tolerances.json, then run with {UpdateVariable}={scenario.Name}.");
        }

        var report = BaselineComparer.Compare(GoldenBaseline.LoadFrom(directory), output);
        Assert.True(report.Passed, report.ToString());
    }

    private static bool UpdateRequested(string name) =>
        (Environment.GetEnvironmentVariable(UpdateVariable) ?? string.Empty)
            .Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries)
            .Contains(name, StringComparer.Ordinal);
}
