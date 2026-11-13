using Tensegrity.Core.Analysis;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The one-input coupled wing (PLAN.md M7) with the study's default
/// geometry: a parallelogram forearm, the elbow driven by a servo and
/// pressed into open and folded stops by an over-center ligament.
/// </summary>
public static class Coupled
{
    /// <summary>The wing resting open with the servo idle and the reference gust at its tip.</summary>
    public static PlanarModel Gusted() =>
        CoupledWing.Create(loads: [CoupledWing.WingtipGust(ReferenceTopologies.WingGustImpulse, ReferenceTopologies.WingGustStart, ReferenceTopologies.WingGustDuration)]);

    /// <summary>The wing resting open, with the reference servo pulses that fold it and open it again.</summary>
    public static PlanarModel Switching() =>
        CoupledWing.Create(servo: CoupledWing.FoldAndOpen(
            ReferenceTopologies.CoupledWingServoTorque,
            ReferenceTopologies.CoupledWingFoldStart,
            ReferenceTopologies.CoupledWingOpenStart,
            ReferenceTopologies.CoupledWingPulseHold,
            ReferenceTopologies.CoupledWingPulseRamp));
}

/// <summary>
/// Reference run for the coupled wing's lock: resting open with the servo
/// idle, hit by the locking wing's gust at its tip, holding. Also records
/// the design the statics produce (ligament rest length, stop preloads,
/// peak servo torque along the fold) and the zero holding torque, so a
/// change to any of them is caught.
/// </summary>
public sealed class CoupledWingScenario : IReferenceScenario
{
    public string Name => "coupled_wing";

    public string Description =>
        "One-input coupled wing (parallelogram forearm: ulna 0.35 m / 0.12 kg, radius 0.35 m / 0.04 kg, hand 0.25 m / 0.06 kg; upper arm bolted), resting open on its elbow stop, pressed there by an over-center ligament, servo idle, standard gravity. At 0.1 s a downward gust of 0.2 N s over 20 ms hits the wingtip; the lock holds. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Coupled.Gusted();
        var output = PlanarRecording.Record(model, model.CreateInitialState(), [CoupledWing.Elbow])
            .ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        output["holding_torque_elbow"] = [model.System.SolveHoldingTorques(0, model.InitialState, [CoupledWing.Elbow])[CoupledWing.Elbow]];

        var geometry = new CoupledWingGeometry();
        var statics = new CoupledWingStatics(geometry);
        output["ligament_rest_length"] = [statics.RestLength];
        output["preload_torque_open"] = [statics.OpenPreload];
        output["preload_torque_folded"] = [statics.FoldPreload];
        output["peak_servo_torque"] = [Enumerable.Range(0, 1501).Max(i => Math.Abs(statics.ServoTorque(geometry.FoldAngle + ((geometry.OpenAngle - geometry.FoldAngle) * i / 1500))))];
        return output;
    }
}

/// <summary>
/// Reference run for the coupled wing's one input: a servo pulse folds the
/// wing, the ligament locks it folded, and a second pulse opens it again.
/// </summary>
public sealed class CoupledWingSwitchScenario : IReferenceScenario
{
    public string Name => "coupled_wing_switch";

    public string Description =>
        "One-input coupled wing as in coupled_wing, without the gust. The elbow servo pushes toward folding with 2 N m for 0.15 s from 0.05 s, then lets go; the ligament carries the elbow on to the folded stop at -150 degrees and holds it. From 0.5 s the same pulse the other way opens the wing again. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Coupled.Switching();
        return PlanarRecording.Record(model, model.CreateInitialState(), [CoupledWing.Elbow]);
    }
}
