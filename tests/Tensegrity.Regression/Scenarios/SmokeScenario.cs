namespace Tensegrity.Regression.Scenarios;

/// <summary>
/// M0 placeholder scenario. There is no physics yet, so this produces a small
/// deterministic set of values whose only purpose is to exercise the
/// load → compare → report path of the regression harness end to end.
/// Physical reference scenarios (free fall, spinning body, pendulum) join the
/// suite in M1.
/// </summary>
public static class SmokeScenario
{
    public const string Name = "smoke";

    public static IReadOnlyDictionary<string, double[]> Run()
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
