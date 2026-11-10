using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Ground contact reference run (PLAN.md 4.1, M6): a bar dropped tilted on
/// to the ground line lands on one end, bounces, slides with Coulomb
/// friction and comes to rest, passing through every contact mode.
/// </summary>
public sealed class GroundDropScenario : IReferenceScenario
{
    public string Name => "ground_drop";

    public string Description =>
        "Slender bar (1 m, 1 kg) with a contact point at each end, dropped on to the ground line y = 0: center 0.4 m up, tilted 30 degrees, 1.5 m/s sideways, turning at -2 rad/s. Penalty normal (k 1e4 N/m, c 60 N s/m), Coulomb friction mu 0.5 through a tangential spring-damper (1e4 N/m, 60 N s/m) in series with a slider. It lands, bounces, slides and stops. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = DroppedBar.Create();
        return PlanarRecording.Record(model, model.CreateInitialState());
    }
}
