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
