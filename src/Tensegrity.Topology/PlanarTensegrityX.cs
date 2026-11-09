using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// The planar two-strut tensegrity (PLAN.md section 8, M2): two struts
/// crossing as the diagonals of a w × h rectangle, held by cables along its
/// sides. The 2D analog of a tensegrity X.
/// <code>
///   TL ───── top ───── TR
///   │ ╲               ╱ │
///   │   ╲ s2     s1 ╱   │
///  left    ╲     ╱    right
///   │        ╲ ╱        │
///   │        ╱ ╲        │
///   │      ╱     ╲      │
///   BL ── bottom ────── BR
/// </code>
/// Corners: BL = (−w/2, 0), BR = (w/2, 0), TL = (−w/2, h), TR = (w/2, h).
/// Strut s1 runs BL → TR and s2 runs BR → TL; each strut's end A is its
/// bottom end. The struts pass through each other: the planar model has no
/// strut contact.
///
/// Rest lengths are worked backwards from the target shape and prestress
/// with <see cref="Cable.RestLengthFor"/>, so the generated pose is an exact
/// equilibrium, not one reached by settling.
/// </summary>
public static class PlanarTensegrityX
{
    public const string Strut1 = "s1";
    public const string Strut2 = "s2";

    /// <summary>
    /// Free-floating X with no gravity and all four cables. Self-equilibrium:
    /// at each corner the two cables balance the strut, so with strut
    /// compression C and diagonal d = √(w² + h²) the horizontal cables carry
    /// C·w/d and the vertical cables C·h/d (T_h / T_v = w / h).
    /// </summary>
    public static PlanarModel FreeFloating(
        double width,
        double height,
        double strutMass,
        double axialStiffness,
        double cableDamping,
        double strutCompression)
    {
        RequirePositive(strutCompression, nameof(strutCompression));
        var (s1, s2, d) = CreateStruts(width, height, strutMass);
        var horizontal = strutCompression * width / d;
        var vertical = strutCompression * height / d;

        Cable Make(string id, Attachment a, Attachment b, double length, double tension) =>
            new(id, a, b, Cable.RestLengthFor(length, tension, axialStiffness), axialStiffness, cableDamping);

        Cable[] cables =
        [
            Make("bottom", Attachment.OnLink(Strut1, s1.EndA), Attachment.OnLink(Strut2, s2.EndA), width, horizontal),
            Make("top", Attachment.OnLink(Strut2, s2.EndB), Attachment.OnLink(Strut1, s1.EndB), width, horizontal),
            Make("left", Attachment.OnLink(Strut1, s1.EndA), Attachment.OnLink(Strut2, s2.EndB), height, vertical),
            Make("right", Attachment.OnLink(Strut2, s2.EndA), Attachment.OnLink(Strut1, s1.EndB), height, vertical),
        ];

        var system = new PlanarSystem([s1, s2], Vector2d.Zero, cables: cables);
        return new PlanarModel("planar_x_free", system, RestState(system, width, height));
    }

    /// <summary>
    /// The X standing under vertical gravity with both strut feet pinned to
    /// the ground at BL and BR. The bottom cable would join two fixed points,
    /// so it is left out; left, right and top remain. Moments about each foot
    /// give, with side-cable tension T_s chosen as the prestress, a top-cable
    /// tension T_t = (w/h)·(T_s + m·g/2). The pins carry the rest.
    /// </summary>
    public static PlanarModel PinnedFeet(
        double width,
        double height,
        double strutMass,
        double axialStiffness,
        double cableDamping,
        double sideCableTension,
        Vector2d gravity)
    {
        RequirePositive(sideCableTension, nameof(sideCableTension));
        if (gravity.X != 0 || !(gravity.Y < 0) || !double.IsFinite(gravity.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(gravity), gravity, "Must point straight down (−y).");
        }

        var (s1, s2, _) = CreateStruts(width, height, strutMass);
        var g = -gravity.Y;
        var top = width / height * (sideCableTension + (strutMass * g / 2));

        Cable Make(string id, Attachment a, Attachment b, double length, double tension) =>
            new(id, a, b, Cable.RestLengthFor(length, tension, axialStiffness), axialStiffness, cableDamping);

        Cable[] cables =
        [
            Make("top", Attachment.OnLink(Strut2, s2.EndB), Attachment.OnLink(Strut1, s1.EndB), width, top),
            Make("left", Attachment.OnLink(Strut1, s1.EndA), Attachment.OnLink(Strut2, s2.EndB), height, sideCableTension),
            Make("right", Attachment.OnLink(Strut2, s2.EndA), Attachment.OnLink(Strut1, s1.EndB), height, sideCableTension),
        ];

        PinJoint[] pins =
        [
            new("foot1", Attachment.OnLink(Strut1, s1.EndA), Attachment.Ground(new Vector2d(-width / 2, 0))),
            new("foot2", Attachment.OnLink(Strut2, s2.EndA), Attachment.Ground(new Vector2d(width / 2, 0))),
        ];

        var system = new PlanarSystem([s1, s2], gravity, pins, cables);
        return new PlanarModel("planar_x", system, RestState(system, width, height));
    }

    private static (PlanarLink S1, PlanarLink S2, double Diagonal) CreateStruts(double width, double height, double strutMass)
    {
        RequirePositive(width, nameof(width));
        RequirePositive(height, nameof(height));
        RequirePositive(strutMass, nameof(strutMass));
        var diagonal = Math.Sqrt((width * width) + (height * height));
        return (
            PlanarLink.SlenderBar(Strut1, strutMass, diagonal, LinkRole.Strut),
            PlanarLink.SlenderBar(Strut2, strutMass, diagonal, LinkRole.Strut),
            diagonal);
    }

    /// <summary>Both struts centered at (0, h/2), at rest: s1 along BL → TR, s2 along BR → TL.</summary>
    private static double[] RestState(PlanarSystem system, double width, double height)
    {
        var state = system.CreateState().Values;
        var center = new Vector2d(0, height / 2);
        system.SetBody(state, 0, new Pose2d(center, Math.Atan2(height, width)), default);
        system.SetBody(state, 1, new Pose2d(center, Math.Atan2(height, -width)), default);
        return state;
    }

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }
}
