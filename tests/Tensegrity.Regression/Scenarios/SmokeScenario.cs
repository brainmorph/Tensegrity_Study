using Tensegrity.Testing;

namespace Tensegrity.Regression.Scenarios;

/// <summary>
/// Harness smoke test with no physics: a small deterministic set of values
/// whose only purpose is to exercise load → compare → report end to end,
/// independently of the physics scenarios. Its baseline was computed in
/// Python, not by this code.
/// </summary>
public sealed class SmokeScenario : IReferenceScenario
{
    public string Name => "smoke";

    public string Description =>
        "M0 harness smoke test. sin(0.1*k) for k=0..9 and H_100 summed in order k=1..100. Values computed independently in Python 3 (math.sin, float repr), not by the C# scenario.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        // sin(0.1·k) for k = 0..9: exercises an array quantity.
        var sineSamples = new double[10];
        for (var k = 0; k < sineSamples.Length; k++)
        {
            sineSamples[k] = Math.Sin(0.1 * k);
        }

        // Harmonic number H_100, summed in a fixed order: exercises a scalar.
        var harmonic = 0.0;
        for (var k = 1; k <= 100; k++)
        {
            harmonic += 1.0 / k;
        }

        return new Dictionary<string, double[]>
        {
            ["sine_samples"] = sineSamples,
            ["harmonic_100"] = [harmonic],
        };
    }
}
