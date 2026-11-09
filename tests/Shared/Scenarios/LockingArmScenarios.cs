using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The two-link locking arm (PLAN.md M5, test 10) with the study's default
/// geometry, and the disturbances used to show it is locked or collapses.
/// </summary>
public static class Arm
{
    /// <summary>Angular rate given to the lower link about the elbow, rad/s; negative droops it.</summary>
    public const double KickRate = -2.0;

    /// <summary>Elbow-tendon rest length multiplier for the collapse case: past the fold at ≈ 1.082.</summary>
    public const double CollapseScale = 1.10;

    /// <summary>The arm held straight out by its tendons, at rest.</summary>
    public static PlanarModel Locked() => LockingArm.Create();

    /// <summary>The arm with its elbow tendon lengthened past the fold, released from straight out.</summary>
    public static PlanarModel Collapsing() => LockingArm.Create(elbowRestLengthScale: CollapseScale);

    /// <summary>
    /// The arm at rest with the upper link at world angle
    /// <paramref name="upperAngle"/> and the lower at
    /// <paramref name="lowerAngle"/>, both pins satisfied exactly, and the
    /// lower link turning about the elbow at <paramref name="lowerRate"/>.
    /// </summary>
    public static SystemState At(PlanarModel model, double upperAngle, double lowerAngle, double lowerRate = 0)
    {
        var system = model.System;
        var upper = system.Links[0];
        var lower = system.Links[1];
        var state = model.CreateInitialState();
        var upperPose = new Pose2d((-upper.EndA).Rotated(upperAngle), upperAngle);
        var elbow = upperPose.ToWorld(upper.EndB);
        var r = (-lower.EndA).Rotated(lowerAngle);
        system.SetBody(state.Values, 0, upperPose, default);
        system.SetBody(state.Values, 1, new Pose2d(elbow + r, lowerAngle), new Twist2d(Vector2d.Cross(lowerRate, r), lowerRate));
        return state;
    }

    /// <summary>The locked arm with the lower link turning about the (still) elbow at <see cref="KickRate"/>.</summary>
    public static SystemState Kicked(PlanarModel model)
    {
        var state = model.CreateInitialState();
        var system = model.System;
        var pose = system.GetPose(state.Values, 1);
        var elbow = system.GetPose(state.Values, 0).ToWorld(system.Links[0].EndB);
        system.SetBody(state.Values, 1, pose, new Twist2d(Vector2d.Cross(KickRate, pose.Position - elbow), KickRate));
        return state;
    }
}

/// <summary>
/// Test 10 (PLAN.md section 7) reference run, locked case: the arm held
/// straight out by its tendons with zero torque, after a kick to the lower
/// link. Also records the zero holding torques and the predicted lock
/// thresholds, so a change to either is caught.
/// </summary>
public sealed class LockingArmScenario : IReferenceScenario
{
    public string Name => "locking_arm";

    public string Description =>
        "Two-link locking arm (upper 0.30 m / 0.20 kg, lower 0.35 m / 0.12 kg), each joint held straight out by one tendon (EA 1000 N, c 150 N s/m) from behind and above it, standard gravity, no torque. Lower link kicked down at 2 rad/s about the elbow; the lock holds. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Arm.Locked();
        var output = PlanarRecording.Record(model, Arm.Kicked(model)).ToDictionary(kv => kv.Key, kv => kv.Value, StringComparer.Ordinal);

        var hold = model.System.SolveHoldingTorques(0, model.InitialState, [LockingArm.Shoulder, LockingArm.Elbow]);
        output["holding_torque_shoulder"] = [hold[LockingArm.Shoulder]];
        output["holding_torque_elbow"] = [hold[LockingArm.Elbow]];

        var statics = ArmStatics.For(model);
        output["lock_threshold_elbow"] = [statics.ElbowFold().Scale];
        output["lock_threshold_shoulder"] = [statics.ShoulderFold().Scale];
        return output;
    }
}

/// <summary>
/// Test 10 reference run, collapsed case: the elbow tendon 10% longer than
/// the lock needs, past its threshold. Released from straight out, the lower
/// link falls and swings through its tendon going slack and taut.
/// </summary>
public sealed class LockingArmCollapseScenario : IReferenceScenario
{
    public string Name => "locking_arm_collapse";

    public string Description =>
        "Two-link locking arm as in locking_arm, but the elbow tendon rest length is 1.10x the design value, past the 1.082 lock threshold. Released at rest from straight out; the lower link collapses and swings. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = Arm.Collapsing();
        return PlanarRecording.Record(model, model.CreateInitialState());
    }
}
