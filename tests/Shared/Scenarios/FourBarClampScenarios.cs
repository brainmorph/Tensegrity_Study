using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>The four-bar toggle clamp (PLAN.md M8) with the study's default geometry, holding its band at the reference tension, and its release twitch.</summary>
public static class Clamp
{
    /// <summary>The clamp latched past center, holding its band, at rest.</summary>
    public static PlanarModel Latched() => FourBarClamp.Create(ReferenceTopologies.ClampTension);

    /// <summary>The latched clamp with the reference release twitch on its crank.</summary>
    public static PlanarModel Released() =>
        FourBarClamp.Create(
            ReferenceTopologies.ClampTension,
            actuators: [FourBarClamp.Twitch(-ReferenceTopologies.ClampRelease, ReferenceTopologies.ClampReleaseStart, ReferenceTopologies.ClampReleaseDuration)]);
}

/// <summary>
/// Over-center reference run, latched case: the clamp holding its band at
/// 50 N, with no torque, its crank pressed into its stop. Also records the
/// toggle from <see cref="FourBarStatics"/>, the stop's torque, and the
/// holding torque around the toggle from the simulator and the statics, so
/// a change to any of them is caught.
/// </summary>
public sealed class FourBarClampScenario : IReferenceScenario
{
    public string Name => "four_bar_clamp";

    public string Description =>
        "Four-bar toggle clamp (crank 5 cm, coupler 17 cm, rocker 10 cm, ground 20 cm), no gravity, holding an elastic band (10 cm, stretched 25%) from the rocker's end at 50 N. Latched 3 degrees past the toggle, the crank rests on its stop with no torque. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Clamp.Latched();
        var output = PlanarRecording.Record(model, model.CreateInitialState(), [FourBarClamp.CrankPivot])
            .ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        var system = model.System;
        output["holding_torque_latched"] = [system.SolveHoldingTorques(0, model.InitialState, [FourBarClamp.CrankPivot])[FourBarClamp.CrankPivot]];
        var stops = new double[system.Pins.Count];
        system.GetLimitTorques(model.InitialState, stops);
        output["stop_torque_latched"] = [stops[0]];

        var statics = new FourBarStatics(new FourBarClampGeometry());
        var (crank, rocker) = statics.Toggle(0.5, 1.6);
        output["toggle_crank"] = [crank];
        output["toggle_rocker"] = [rocker];

        // Holding torque against the band, clear of the stop: 3° short of the toggle, at it, and 2° past it.
        double[] offsets = [-3, 0, 2];
        var simulated = new double[offsets.Length];
        var predicted = new double[offsets.Length];
        for (var i = 0; i < offsets.Length; i++)
        {
            var theta = crank + (offsets[i] * Math.PI / 180);
            var posed = FourBarClamp.Create(ReferenceTopologies.ClampTension, crankAngle: theta);
            simulated[i] = posed.System.SolveHoldingTorques(0, posed.InitialState, [FourBarClamp.CrankPivot])[FourBarClamp.CrankPivot];
            predicted[i] = statics.BandHoldingTorque(theta, statics.RockerAngle(theta, rocker), ReferenceTopologies.ClampTension);
        }

        output["holding_torque_crank"] = simulated;
        output["predicted_holding_torque_crank"] = predicted;
        return output;
    }
}

/// <summary>
/// Over-center reference run, released case: the latched clamp, then a
/// 20 ms twitch pulls its crank back over the toggle, and the band throws
/// it open against its open stop and, still stretched, holds it there.
/// </summary>
public sealed class FourBarClampReleaseScenario : IReferenceScenario
{
    public string Name => "four_bar_clamp_release";

    public string Description =>
        "Four-bar toggle clamp as in four_bar_clamp. At 0.1 s a 20 ms twitch of 0.003 N m s pulls the crank back over the toggle, and the band throws the clamp open against its open stop 30 degrees short of the toggle, where it holds it. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Clamp.Released();
        return PlanarRecording.Record(model, model.CreateInitialState(), [FourBarClamp.CrankPivot]);
    }
}
