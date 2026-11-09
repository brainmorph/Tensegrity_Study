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
