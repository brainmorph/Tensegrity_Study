using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Perturbation reference run (PLAN.md M6): a bar at rest on the ground,
/// disturbed by a ground drop and then a knock at one end.
/// </summary>
public sealed class PerturbedBarScenario : IReferenceScenario
{
    public string Name => "perturbed_bar";

    public string Description =>
        "Slender bar (1 m, 1 kg) lying at rest on the ground line (as in ground_drop). At 0.1 s the ground falls 0.1 m in 10 ms and the bar falls after it and lands; at 0.5 s a 5 ms triangular pulse of (0.3, 0.6) N s at its right end knocks it up and sideways; it rocks, lands and settles. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = PerturbedBar.Create();
        return PlanarRecording.Record(model, model.CreateInitialState());
    }
}
