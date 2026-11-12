using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Topology;

/// <summary>
/// Dimensions of a <see cref="ThreeStrut"/> frame, SI units. The muscle's
/// anchor, tendon and stop were chosen by the M9 statics search: the tendon
/// stays under 8% strain, and the muscle ends at 5% of its starting tension.
/// </summary>
/// <param name="PostLength">Each post's length, foot to top, m.</param>
/// <param name="PostMass">Each post's mass, kg.</param>
/// <param name="Spacing">Distance between neighboring feet, m. The feet are at x = 0, s, 2s.</param>
/// <param name="Lean">How far the posts lean left in the starting shape, rad.</param>
/// <param name="StayAnchorX">The stay's ground anchor, x, m. The stay holds the starting shape from the right.</param>
/// <param name="GuyAnchorX">The guy's ground anchor, x, m. The guy catches the snapped shape from the left.</param>
/// <param name="MuscleAnchorX">The muscle's ground anchor, x, m: just right of the third foot, for a short moment arm.</param>
/// <param name="CableAxialStiffness">EA of the top cables, stay and guy, N.</param>
/// <param name="CableDamping">Damping c of the top cables, stay and guy, N·s/m.</param>
/// <param name="MuscleTension">The muscle's tension in the starting shape, N.</param>
/// <param name="TendonAxialStiffness">The muscle tendon's EA, N.</param>
/// <param name="TendonDamping">The muscle tendon's damping c, N·s/m.</param>
/// <param name="ContractileLength">The muscle's contractile length at the start, m.</param>
/// <param name="ShorteningSpeed">The muscle's shortening speed at full activation, m/s.</param>
/// <param name="ActivationStart">When the muscle starts to activate, s.</param>
/// <param name="ActivationRamp">How long the activation takes to rise to full, and to fall back, s.</param>
/// <param name="Shortening">How far the muscle shortens in all, m. The statics put the stop where the snapped shape leaves the muscle at 5% of its starting tension.</param>
public sealed record ThreeStrutGeometry(
    double PostLength = 1.0,
    double PostMass = 0.5,
    double Spacing = 0.4,
    double Lean = 15 * Math.PI / 180,
    double StayAnchorX = 1.6,
    double GuyAnchorX = -0.8,
    double MuscleAnchorX = 0.9,
    double CableAxialStiffness = 1000.0,
    double CableDamping = 20.0,
    double MuscleTension = 5.0,
    double TendonAxialStiffness = 360.0,
    double TendonDamping = 20.0,
    double ContractileLength = 0.3,
    double ShorteningSpeed = 0.01,
    double ActivationStart = 0.5,
    double ActivationRamp = 0.1,
    double Shortening = 0.04203343);

/// <summary>
/// A three-strut frame that snaps through when one muscle shortens (PLAN.md
/// M9). Three posts stand on pins at their feet, joined at their tops by two
/// cables, so they lean together. They touch nothing but cables and their
/// pins.
/// <code>
///   guy        top1 ── c12 ── top2 ── c23 ── top3
///     ╲         ╱             ╱             ╱ │╲  stay
///      ╲     post1         post2         post3│  ╲
///       ╲     ╱             ╱             ╱   │muscle
///   ─────●───○─────────────○─────────────○────●───────●───
///      guy  foot1        foot2         foot3 muscle  stay
///    anchor                                  anchor  anchor
/// </code>
/// In the starting shape the posts lean left by <see cref="ThreeStrutGeometry.Lean"/>.
/// Gravity tips them further left; the stay and the muscle, both from the
/// third top to anchors on the right, hold them. The muscle runs nearly
/// along the third post, to an anchor just past its foot, so it has a short
/// moment arm, like a muscle inserted near a joint: high force, small travel.
///
/// As the muscle shortens, its tension rises and the stay unloads. When the
/// stay goes slack, nothing but the muscle holds the frame, and the muscle's
/// tendon is too soft to: the frame falls over center to the right, where
/// the guy catches it. Leaning right, the third top has swung toward the
/// muscle's anchor, so the muscle's path is shorter and its tendon relaxes.
/// The muscle stops at the length where that leaves it at 5% of its
/// starting tension, and the snapped shape holds on the guy.
///
/// Rest lengths are worked back from the starting shape by moment balance,
/// so it is an exact equilibrium. With level tops, the top cables carry
/// m·g·tan λ / 2 and m·g·tan λ, and the stay and muscle together hold
/// 3·m·g·(l/2)·sin λ about the third foot. The guy is tensioned as the
/// stay's mirror image, holding the same moment from the left when the
/// posts lean right by λ. Each post's end A is its foot.
/// </summary>
public static class ThreeStrut
{
    public static readonly string[] Posts = ["post1", "post2", "post3"];

    public const string Stay = "stay";
    public const string Guy = "guy";
    public const string Muscle = "muscle";

    /// <summary>The frame in its starting shape, at rest, with the muscle about to shorten.</summary>
    public static PlanarModel Create(ThreeStrutGeometry? geometry = null)
    {
        var g = geometry ?? new ThreeStrutGeometry();
        RequirePositive(g.PostLength, nameof(g.PostLength));
        RequirePositive(g.PostMass, nameof(g.PostMass));
        RequirePositive(g.Spacing, nameof(g.Spacing));
        RequirePositive(g.Lean, nameof(g.Lean));
        var gravity = PlanarSystem.StandardGravity;
        var weight = -g.PostMass * gravity.Y;

        var links = Posts.Select(id => PlanarLink.SlenderBar(id, g.PostMass, g.PostLength, LinkRole.Strut)).ToArray();
        var post = links[0];
        var tops = Enumerable.Range(0, 3).Select(i => Top(g, i, -g.Lean)).ToArray();

        // Level tops: each top cable holds up the posts to its left.
        var t12 = weight * Math.Tan(g.Lean) / 2;
        var t23 = weight * Math.Tan(g.Lean);
        var moment = 3 * weight * (g.PostLength / 2) * Math.Sin(g.Lean);

        var stayAnchor = new Vector2d(g.StayAnchorX, 0);
        var guyAnchor = new Vector2d(g.GuyAnchorX, 0);
        var muscleAnchor = new Vector2d(g.MuscleAnchorX, 0);
        var stayTension = (moment - (g.MuscleTension * ClockwiseArm(g, 2, -g.Lean, muscleAnchor))) / ClockwiseArm(g, 2, -g.Lean, stayAnchor);
        if (!(stayTension > 0))
        {
            throw new ArgumentException($"The muscle alone holds more than the starting shape needs, leaving the stay {stayTension} N.", nameof(geometry));
        }

        var guyLength = (Top(g, 0, g.Lean) - guyAnchor).Length;
        var guyTension = moment / -ClockwiseArm(g, 0, g.Lean, guyAnchor);
        var tendonLength = (tops[2] - muscleAnchor).Length - g.ContractileLength;

        Attachment TopOf(int i) => Attachment.OnLink(Posts[i], post.EndB);
        Cable Plain(string id, Attachment a, Attachment b, double length, double tension) =>
            new(id, a, b, Cable.RestLengthFor(length, tension, g.CableAxialStiffness), g.CableAxialStiffness, g.CableDamping);

        Cable[] cables =
        [
            Plain("c12", TopOf(0), TopOf(1), g.Spacing, t12),
            Plain("c23", TopOf(1), TopOf(2), g.Spacing, t23),
            Plain(Stay, TopOf(2), Attachment.Ground(stayAnchor), (tops[2] - stayAnchor).Length, stayTension),
            Plain(Guy, TopOf(0), Attachment.Ground(guyAnchor), guyLength, guyTension),
            new(
                Muscle,
                TopOf(2),
                Attachment.Ground(muscleAnchor),
                Cable.RestLengthFor(tendonLength, g.MuscleTension, g.TendonAxialStiffness),
                g.TendonAxialStiffness,
                g.TendonDamping,
                new Muscle(g.ContractileLength, g.ShorteningSpeed, Activation(g))),
        ];

        var pins = Enumerable.Range(0, 3)
            .Select(i => new PinJoint($"foot{i + 1}", Attachment.OnLink(Posts[i], post.EndA), Attachment.Ground(Foot(g, i))))
            .ToArray();

        var system = new PlanarSystem(links, gravity, pins, cables);
        var state = system.CreateState().Values;
        for (var i = 0; i < 3; i++)
        {
            var center = Foot(g, i) + ((g.PostLength / 2) * Direction(-g.Lean));
            system.SetBody(state, i, new Pose2d(center, (Math.PI / 2) + g.Lean), default);
        }

        return new PlanarModel("three_strut_snap", system, state);
    }

    /// <summary>
    /// The muscle's activation: off until <see cref="ThreeStrutGeometry.ActivationStart"/>,
    /// up to full over one ramp, held, and back to zero over another, so it
    /// shortens by <see cref="ThreeStrutGeometry.Shortening"/> in all.
    /// </summary>
    public static Schedule Activation(ThreeStrutGeometry geometry)
    {
        var g = geometry ?? throw new ArgumentNullException(nameof(geometry));
        var hold = (g.Shortening / g.ShorteningSpeed) - g.ActivationRamp;
        if (!(hold > 0))
        {
            throw new ArgumentException("The shortening is too short for the activation's ramps.", nameof(geometry));
        }

        var on = g.ActivationStart + g.ActivationRamp;
        return new Schedule(
        [
            new SchedulePoint(g.ActivationStart, 0),
            new SchedulePoint(on, 1),
            new SchedulePoint(on + hold, 1),
            new SchedulePoint(on + hold + g.ActivationRamp, 0),
        ]);
    }

    /// <summary>Foot of post <paramref name="i"/>: x = i·s on the ground.</summary>
    public static Vector2d Foot(ThreeStrutGeometry g, int i) => new(i * g.Spacing, 0);

    /// <summary>Top of post <paramref name="i"/> leaning right by <paramref name="lean"/> (negative leans left).</summary>
    public static Vector2d Top(ThreeStrutGeometry g, int i, double lean) => Foot(g, i) + (g.PostLength * Direction(lean));

    private static Vector2d Direction(double lean) => new(Math.Sin(lean), Math.Cos(lean));

    /// <summary>Moment arm, clockwise positive, about post <paramref name="i"/>'s foot of a unit pull from its top toward <paramref name="anchor"/>.</summary>
    private static double ClockwiseArm(ThreeStrutGeometry g, int i, double lean, Vector2d anchor)
    {
        var top = Top(g, i, lean);
        var r = top - Foot(g, i);
        var pull = (anchor - top) / (anchor - top).Length;
        return -((r.X * pull.Y) - (r.Y * pull.X));
    }

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }
}
