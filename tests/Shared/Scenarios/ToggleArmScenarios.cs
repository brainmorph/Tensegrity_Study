using Tensegrity.Core.Elements;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The two-link toggle arm (PLAN.md M8) with the study's default geometry,
/// its two poles and barrier from <see cref="ChainStatics"/>, and the elbow
/// twitches used to show it holds or snaps.
/// </summary>
public static class Toggle
{
    /// <summary>The toggle arm latched straight, at rest, with no twitch.</summary>
    public static PlanarModel Latched() => ToggleArm.Create();

    /// <summary>A 20 ms elbow twitch of the given angular impulse starting at <paramref name="start"/>.</summary>
    public static JointActuator Twitch(double impulse, double start) =>
        ToggleArm.Twitch(impulse, start, ReferenceTopologies.TwitchDuration);

    /// <summary>The latched arm with a folding twitch under what snaps it.</summary>
    public static PlanarModel Held() =>
        ToggleArm.Create(actuators: [Twitch(ReferenceTopologies.FoldTwitchUnder, ReferenceTopologies.FoldTwitchStart)]);

    /// <summary>The latched arm snapped to folded by one twitch and back to straight by another.</summary>
    public static PlanarModel Snapping() =>
        ToggleArm.Create(actuators:
        [
            ToggleArm.Twitches(
                [(ReferenceTopologies.FoldTwitchOver, ReferenceTopologies.FoldTwitchStart), (ReferenceTopologies.ExtendTwitchOver, ReferenceTopologies.ExtendTwitchStart)],
                ReferenceTopologies.TwitchDuration),
        ]);

    /// <summary>The statics of the default toggle arm.</summary>
    public static ChainStatics Statics() => ChainStatics.ForToggle(Latched());

    /// <summary>The straight pole (world link angles): exactly straight out, by construction.</summary>
    public static double[] StraightPole(ChainStatics statics) => statics.RestPoseNear([0, 0], [1, 1]);

    /// <summary>The folded pole (world link angles), pressed into the fold stop.</summary>
    public static double[] FoldedPole(ChainStatics statics) => statics.RestPoseNear([0, -(Math.PI / 3) - 0.05], [1, 1]);
}

/// <summary>
/// Snap-latch reference run, held case: the toggle arm latched straight,
/// given a folding twitch at the elbow 6% under what snaps it. The elbow
/// bends to about −30° and comes back. Also records the zero
/// holding torques, the stop's press, and the predicted poles and barrier,
/// so a change to any of them is caught.
/// </summary>
public sealed class ToggleArmScenario : IReferenceScenario
{
    public string Name => "toggle_arm";

    public string Description =>
        "Two-link toggle arm (upper 0.30 m / 0.20 kg, lower 0.35 m / 0.12 kg), shoulder held by a tendon, elbow by an over-center tendon (EA 1000 N, c 150 N s/m) pulling it into a stop at straight or at -60 degrees; standard gravity, no torque at rest. At 0.1 s a 20 ms folding twitch of 0.070 N m s across the elbow; the latch holds. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Toggle.Held();
        var output = PlanarRecording.Record(model, model.CreateInitialState(), [ToggleArm.Elbow])
            .ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        var system = model.System;
        var hold = system.SolveHoldingTorques(0, model.InitialState, [ToggleArm.Shoulder, ToggleArm.Elbow]);
        output["holding_torque_shoulder"] = [hold[ToggleArm.Shoulder]];
        output["holding_torque_elbow"] = [hold[ToggleArm.Elbow]];
        var stops = new double[system.Pins.Count];
        system.GetLimitTorques(model.InitialState, stops);
        output["straight_stop_torque"] = [stops[1]];

        var statics = Toggle.Statics();
        var straight = Toggle.StraightPole(statics);
        var folded = Toggle.FoldedPole(statics);
        var barrier = statics.Barrier(1, straight, folded);
        output["folded_pole_upper"] = [folded[0]];
        output["folded_pole_elbow"] = [folded[1] - folded[0]];
        output["barrier_elbow"] = [barrier.JointAngle];
        output["barrier_from_straight"] = [barrier.Potential - statics.Potential(straight, [1, 1])];
        output["barrier_from_folded"] = [barrier.Potential - statics.Potential(folded, [1, 1])];
        return output;
    }
}

/// <summary>
/// Snap-latch reference run, snap case: a folding twitch 7% over the
/// threshold snaps the straight elbow into its fold stop, where it stays
/// with no torque; an extending twitch 9% over its threshold snaps it back.
/// </summary>
public sealed class ToggleArmSnapScenario : IReferenceScenario
{
    public string Name => "toggle_arm_snap";

    public string Description =>
        "Two-link toggle arm as in toggle_arm. At 0.1 s a 20 ms folding twitch of 0.080 N m s across the elbow snaps it into the fold stop at -60 degrees; at 0.55 s an extending twitch of 0.056 N m s snaps it back to straight. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Toggle.Snapping();
        return PlanarRecording.Record(model, model.CreateInitialState(), [ToggleArm.Elbow]);
    }
}
