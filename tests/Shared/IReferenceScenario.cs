namespace Tensegrity.Testing;

/// <summary>
/// A deterministic scenario whose named outputs are checked against a golden
/// baseline in tests/golden/&lt;Name&gt;/ by the regression suite.
/// </summary>
public interface IReferenceScenario
{
    /// <summary>Baseline folder name; snake_case.</summary>
    string Name { get; }

    /// <summary>One or two sentences stored in baseline.json.</summary>
    string Description { get; }

    IReadOnlyDictionary<string, double[]> Run();
}
