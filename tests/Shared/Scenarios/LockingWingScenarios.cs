using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The three-link locking wing (PLAN.md M6, test 10 extended to a wrist)
/// with the study's default geometry, and the poses used to show it is
/// locked or collapses.
/// </summary>
public static class Wing
{
    /// <summary>The wing held straight out by its tendons, at rest.</summary>
    public static PlanarModel Locked() => LockingWing.Create();

    /// <summary>The straight wing with the reference gust at its tip.</summary>
    public static PlanarModel Gusted() =>
        LockingWing.Create(loads: [LockingWing.WingtipGust(ReferenceTopologies.WingGustImpulse, ReferenceTopologies.WingGustStart, ReferenceTopologies.WingGustDuration)]);

    /// <summary>The straight wing preloaded against hyperextension stops at the elbow and wrist, at rest.</summary>
    public static PlanarModel Preloaded() => LockingWing.Create(ReferenceTopologies.PreloadedWing);

    /// <summary>The preloaded wing with the reference gust at its tip.</summary>
    public static PlanarModel PreloadedGusted() =>
        LockingWing.Create(ReferenceTopologies.PreloadedWing, loads: [LockingWing.WingtipGust(ReferenceTopologies.WingGustImpulse, ReferenceTopologies.WingGustStart, ReferenceTopologies.WingGustDuration)]);

    /// <summary>The wing with its wrist tendon lengthened past the fold, released from straight out.</summary>
    public static PlanarModel Collapsing() => LockingWing.Create(restLengthScales: [1, 1, ReferenceTopologies.WingCollapseScale]);

    /// <summary>
    /// The wing at rest with its links at world angles
    /// <paramref name="angles"/>, every pin satisfied exactly, and the hand
    /// turning about the (still) wrist at <paramref name="handRate"/>.
    /// </summary>
    public static SystemState At(PlanarModel model, IReadOnlyList<double> angles, double handRate = 0)
    {
        var system = model.System;
        var state = model.CreateInitialState();
        var joint = Vector2d.Zero;
        for (var b = 0; b < 3; b++)
        {
            var link = system.Links[b];
            var r = (-link.EndA).Rotated(angles[b]);
            var rate = b == 2 ? handRate : 0;
            system.SetBody(state.Values, b, new Pose2d(joint + r, angles[b]), new Twist2d(Vector2d.Cross(rate, r), rate));
            joint += 2 * r;
        }

        return state;
    }
}

/// <summary>
/// Test 10 reference run for the wing, locked case: the straight wing hit
/// by a gust at its tip, holding. Also records the zero holding torques and
/// the three predicted lock thresholds, so a change to either is caught.
/// </summary>
public sealed class LockingWingScenario : IReferenceScenario
{
    public string Name => "locking_wing";

    public string Description =>
        "Three-link locking wing (upper 0.30 m / 0.20 kg, lower 0.35 m / 0.12 kg, hand 0.25 m / 0.06 kg), each joint held straight out by one tendon from behind and above it, standard gravity, no torque. At 0.1 s a downward gust of 0.2 N s over 20 ms hits the wingtip; the lock holds. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Wing.Gusted();
        var output = PlanarRecording.Record(model, model.CreateInitialState(), [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist])
            .ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        var hold = model.System.SolveHoldingTorques(0, model.InitialState, [LockingWing.Shoulder, LockingWing.Elbow, LockingWing.Wrist]);
        output["holding_torque_shoulder"] = [hold[LockingWing.Shoulder]];
        output["holding_torque_elbow"] = [hold[LockingWing.Elbow]];
        output["holding_torque_wrist"] = [hold[LockingWing.Wrist]];

        var statics = ChainStatics.ForWing(Wing.Locked());
        output["lock_threshold_shoulder"] = [statics.Fold(0).Scale];
        output["lock_threshold_elbow"] = [statics.Fold(1).Scale];
        output["lock_threshold_wrist"] = [statics.Fold(2).Scale];
        return output;
    }
}

/// <summary>
/// Test 10 reference run for the wing, collapsed case: the wrist tendon 8%
/// longer than the lock needs, past its threshold. Released from straight
/// out, the hand falls and swings, hanging from the wrist.
/// </summary>
public sealed class LockingWingCollapseScenario : IReferenceScenario
{
    public string Name => "locking_wing_collapse";

    public string Description =>
        "Three-link locking wing as in locking_wing, without the gust, but the wrist tendon rest length is 1.08x the design value, past the 1.063 lock threshold. Released at rest from straight out; the hand collapses and swings from the wrist. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Wing.Collapsing();
        return PlanarRecording.Record(model, model.CreateInitialState(), [LockingWing.Wrist]);
    }
}
