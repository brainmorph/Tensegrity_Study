using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Snap-through reference run (PLAN.md M9): the <see cref="ThreeStrut"/>
/// frame with its default geometry, from its starting lean through the
/// muscle's slow shortening, the snap, and the ring-down on the guy. Also
/// records the statics' predictions, so a change to the snap or the snapped
/// shape is caught even where the run's samples fall between them.
/// </summary>
public sealed class ThreeStrutSnapScenario : IReferenceScenario
{
    /// <summary>10 s at dt 1e-4 s: the snap is at about 3.5 s and the frame is still by 7 s.</summary>
    public const long Steps = 100_000;

    /// <summary>Sampled every 0.25 s.</summary>
    public const long Stride = 2_500;

    public string Name => "three_strut_snap";

    public string Description =>
        "Three posts (1 m / 0.5 kg) pinned at their feet 0.4 m apart and joined at the top by two cables, standard gravity, no torque. Leaning 15 degrees left, held by a stay and by a series-elastic muscle (tendon EA 360 N, c 20 N s/m, 5 N) along the third post. From 0.5 s the muscle shortens at 1 cm/s; its tension rises to about 19.7 N, the stay goes slack and the frame snaps over center onto a guy at 15 degrees right. The muscle stops 4.2 cm in, leaving it at 5% of its starting tension. RK4 with event splitting, dt 1e-4 s, 10 s, sampled every 0.25 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = ThreeStrut.Create();
        var output = PlanarRecording.Record(model, model.CreateInitialState(), steps: Steps, stride: Stride)
            .ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        var geometry = new ThreeStrutGeometry();
        var statics = new ThreeStrutStatics(geometry, model.System.Cables);
        var snap = statics.FindSnap();
        var snapped = statics.Snapped(geometry.Shortening);
        output["snap_shortening"] = [snap.Shortening];
        output["snap_muscle_tension"] = [snap.MuscleTension];
        output["snap_leans"] = snap.Leans;
        output["snapped_leans"] = snapped;
        output["snapped_muscle_tension"] = [statics.MuscleTension(snapped, geometry.Shortening)];
        return output;
    }
}
