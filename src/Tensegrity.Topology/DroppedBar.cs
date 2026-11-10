using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// A slender bar dropped on to the ground line (PLAN.md 4.1, M6): the
/// reference case for ground contact. It has a contact point at each end
/// and is released tilted, moving sideways and turning, so it lands on one
/// end, bounces, slides and comes to rest lying flat. On the way it goes
/// through every normal mode (clear, pressing, recoiling) and every
/// friction mode (free, stick, slip both ways).
/// </summary>
public static class DroppedBar
{
    public const string Bar = "bar";
    public const string Left = "left";
    public const string Right = "right";

    /// <summary>
    /// The bar with its center at <paramref name="height"/> above the line
    /// y = 0, at angle <paramref name="angle"/>, moving at
    /// <paramref name="vx"/> and turning at <paramref name="omega"/>.
    /// The ground parameters give a contact frequency of about 140 rad/s
    /// for a 1 kg bar on both ends, well resolved at dt = 1e-4 s, with a
    /// normal damping ratio of 0.3.
    /// </summary>
    public static PlanarModel Create(
        double mass = 1.0,
        double length = 1.0,
        double normalStiffness = 1e4,
        double normalDamping = 60.0,
        double friction = 0.5,
        double tangentialStiffness = 1e4,
        double tangentialDamping = 60.0,
        double height = 0.4,
        double angle = Math.PI / 6,
        double vx = 1.5,
        double omega = -2.0)
    {
        var bar = PlanarLink.SlenderBar(Bar, mass, length);
        var ground = new GroundLine(
            [new ContactPoint(Left, Bar, bar.EndA), new ContactPoint(Right, Bar, bar.EndB)],
            normalStiffness,
            normalDamping,
            friction,
            tangentialStiffness,
            tangentialDamping);
        var system = new PlanarSystem([bar], PlanarSystem.StandardGravity, ground: ground);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, new Pose2d(new Vector2d(0, height), angle), new Twist2d(new Vector2d(vx, 0), omega));
        return new PlanarModel("ground_drop", system, state);
    }
}
