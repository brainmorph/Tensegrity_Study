using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// The perturbation reference case (PLAN.md M6): the <see cref="DroppedBar"/>
/// bar lying at rest on the ground line, then disturbed twice. First the
/// ground falls away (a ground drop) and the bar falls after it and lands;
/// then a short force pulse knocks one end up and sideways.
/// </summary>
public static class PerturbedBar
{
    public const string Knock = "knock";

    /// <summary>When the ground starts to fall, s; it falls <see cref="DropDepth"/> in <see cref="DropDuration"/>.</summary>
    public const double DropStart = 0.1;

    public const double DropDuration = 0.01;

    public const double DropDepth = 0.1;

    /// <summary>When the knock starts, s; a triangular pulse of <see cref="KnockDuration"/>.</summary>
    public const double KnockStart = 0.5;

    public const double KnockDuration = 0.005;

    /// <summary>The knock's impulse at the bar's right end, N·s.</summary>
    public static readonly Vector2d KnockImpulse = new(0.3, 0.6);

    public static PlanarModel Create()
    {
        const double normalStiffness = 1e4;
        var bar = PlanarLink.SlenderBar(DroppedBar.Bar, 1.0, 1.0);
        var ground = new GroundLine(
            [new ContactPoint(DroppedBar.Left, DroppedBar.Bar, bar.EndA), new ContactPoint(DroppedBar.Right, DroppedBar.Bar, bar.EndB)],
            normalStiffness,
            normalDamping: 60.0,
            friction: 0.5,
            tangentialStiffness: 1e4,
            tangentialDamping: 60.0,
            heightSchedule: Schedule.Ramp(DropStart, 0.0, DropStart + DropDuration, -DropDepth));
        var knock = ExternalLoad.Pulse(Knock, DroppedBar.Bar, bar.EndB, KnockImpulse, KnockStart, KnockDuration);
        var system = new PlanarSystem([bar], PlanarSystem.StandardGravity, ground: ground, loads: [knock]);

        // Lying flat at rest, each end sunk to carry half the weight.
        var state = system.CreateState().Values;
        var sink = bar.Mass * -PlanarSystem.StandardGravity.Y / (2 * normalStiffness);
        system.SetBody(state, 0, new Pose2d(new Vector2d(0, -sink), 0), default);
        return new PlanarModel("perturbed_bar", system, state);
    }
}
