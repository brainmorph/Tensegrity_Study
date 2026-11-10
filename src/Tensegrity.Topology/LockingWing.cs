using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// Dimensions of a <see cref="LockingWing"/>, SI units. The upper and lower
/// links and their tendons match <see cref="LockingArmGeometry"/>'s
/// defaults; the hand is new. Generic, roughly bird-wing scale (PLAN.md 2:
/// no specific bird).
/// </summary>
/// <param name="Arm">Upper and lower links, shoulder and elbow tendons, and the elbow fold stop.</param>
/// <param name="HandLength">Hand (wrist outward), m.</param>
/// <param name="HandMass">Hand mass, kg.</param>
/// <param name="WristPostBehind">How far the wrist tendon's post sits behind the wrist, along the lower link, m.</param>
/// <param name="WristPostAbove">How far the post sits above the lower link's axis, m.</param>
/// <param name="WristInsertion">Where the wrist tendon attaches to the hand, measured from the wrist, m.</param>
/// <param name="WristFoldStop">How far the wrist may fold downward from straight before the bones meet, rad; null for no stop.</param>
/// <param name="WristStopStiffness">Torsional stiffness of the wrist fold stop, N·m/rad.</param>
/// <param name="WristStopDamping">Torsional damping of the wrist fold stop, N·m·s/rad: about 30% of critical against the hand.</param>
/// <param name="HyperextensionStops">
/// Whether the elbow and wrist have stops against bending upward past the
/// design pose, with the same stiffness and damping as their fold stops.
/// The shoulder has none.
/// </param>
/// <param name="ElbowPreload">
/// Torque the elbow's hyperextension stop carries in the design pose, N·m:
/// the elbow tendon is tensioned to pull the lower link into its stop this
/// hard on top of holding its weight. Needs <paramref name="HyperextensionStops"/>.
/// </param>
/// <param name="WristPreload">The same for the wrist, N·m.</param>
public sealed record LockingWingGeometry(
    LockingArmGeometry? Arm = null,
    double HandLength = 0.25,
    double HandMass = 0.06,
    double WristPostBehind = 0.03,
    double WristPostAbove = 0.03,
    double WristInsertion = 0.07,
    double? WristFoldStop = 5 * Math.PI / 6,
    double WristStopStiffness = 5.0,
    double WristStopDamping = 0.05,
    bool HyperextensionStops = false,
    double ElbowPreload = 0,
    double WristPreload = 0)
{
    /// <summary>The arm part, with <see cref="LockingArmGeometry"/>'s defaults if none was given.</summary>
    public LockingArmGeometry ArmGeometry => Arm ?? new LockingArmGeometry();
}

/// <summary>
/// The three-link locking wing (PLAN.md M6): the <see cref="LockingArm"/>
/// with a hand pinned to the lower link at the wrist, held by a third
/// tendon in the same arrangement as the elbow's.
/// <code>
///   anchor ●            post ●               post ●
///           ╲ shoulder       ╲ elbow              ╲ wrist
///            ╲ tendon         ╲ tendon             ╲ tendon
///   shoulder ○━━━━ upper ━━━━━○━━━━━━ lower ━━━━━━━━○━━━━ hand ━━━━
///                            elbow                 wrist
/// </code>
/// Each tendon runs from a point behind and above its joint to a point
/// further out on the next link, and is tensioned so the chosen pose is an
/// exact zero-torque equilibrium: the wrist tendon carries the hand's
/// gravity moment about the wrist, the elbow tendon that of the lower link
/// and hand about the elbow, and the shoulder tendon that of all three about
/// the shoulder. The elbow and wrist have fold stops so a collapsing link
/// cannot fold through the one before it.
///
/// Optionally the elbow and wrist also have hyperextension stops, and their
/// tendons can be designed shorter so they preload each joint up into its
/// stop. A penalty stop pushes only once the joint is past it, so a stop
/// preloaded by P sits P/k short of the design pose (k its stiffness): in
/// that pose it carries exactly P and the pose stays an exact equilibrium.
/// Gravity pulls the joints away from these stops, so without a preload
/// they only catch the wing flicking up.
///
/// The shoulder is at the origin. Pin ends A are the outer links, so a
/// positive holding torque lifts the wing; each joint angle is the outer
/// link's world angle minus the inner one's.
/// </summary>
public static class LockingWing
{
    public const string Upper = LockingArm.Upper;
    public const string Lower = LockingArm.Lower;
    public const string Hand = "hand";
    public const string Shoulder = LockingArm.Shoulder;
    public const string Elbow = LockingArm.Elbow;
    public const string Wrist = "wrist";
    public const string ShoulderTendon = LockingArm.ShoulderTendon;
    public const string ElbowTendon = LockingArm.ElbowTendon;
    public const string WristTendon = "wrist_tendon";

    /// <summary>Id of the wingtip gust in the reference case.</summary>
    public const string Gust = "gust";

    /// <summary>
    /// A downward gust at the wingtip: a triangular force pulse of
    /// <paramref name="impulse"/> N·s starting at <paramref name="start"/>
    /// and lasting <paramref name="duration"/>, at the hand's outer end.
    /// </summary>
    public static ExternalLoad WingtipGust(double impulse, double start, double duration, LockingWingGeometry? geometry = null)
    {
        var w = geometry ?? new LockingWingGeometry();
        return ExternalLoad.Pulse(Gust, Hand, new Vector2d(w.HandLength / 2, 0), new Vector2d(0, -impulse), start, duration);
    }

    /// <summary>
    /// The wing at rest with its links at world angles
    /// <paramref name="angles"/> (upper, lower, hand; 0 is straight out
    /// along +x, all zero if omitted). The tendons are tensioned to hold
    /// that pose, then their rest lengths are multiplied by
    /// <paramref name="restLengthScales"/> (shoulder, elbow, wrist; 1 keeps
    /// the exact equilibrium, more than 1 lengthens a tendon). Optional
    /// <paramref name="loads"/> perturb it, such as a gust at the wingtip.
    /// </summary>
    public static PlanarModel Create(
        LockingWingGeometry? geometry = null,
        IReadOnlyList<double>? angles = null,
        IReadOnlyList<double>? restLengthScales = null,
        Vector2d? gravity = null,
        IEnumerable<ExternalLoad>? loads = null)
    {
        var w = geometry ?? new LockingWingGeometry();
        var g = w.ArmGeometry;
        var grav = gravity ?? PlanarSystem.StandardGravity;
        var theta = angles ?? [0, 0, 0];
        var scales = restLengthScales ?? [1, 1, 1];
        if (theta.Count != 3 || scales.Count != 3)
        {
            throw new ArgumentException("A wing has three links and three tendons.");
        }

        foreach (var s in scales)
        {
            if (!double.IsFinite(s) || s <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(restLengthScales), s, "Must be finite and positive.");
            }
        }

        if (!(w.WristInsertion > 0 && w.WristInsertion <= w.HandLength))
        {
            throw new ArgumentOutOfRangeException(nameof(geometry), w.WristInsertion, $"The wrist insertion must lie on the hand, in (0, {w.HandLength}].");
        }

        foreach (var (name, preload) in new[] { (nameof(w.ElbowPreload), w.ElbowPreload), (nameof(w.WristPreload), w.WristPreload) })
        {
            if (!double.IsFinite(preload) || preload < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(geometry), preload, $"{name} must be finite and not negative.");
            }

            if (preload > 0 && !w.HyperextensionStops)
            {
                throw new ArgumentException($"{name} needs hyperextension stops to push against.", nameof(geometry));
            }
        }

        var upper = PlanarLink.SlenderBar(Upper, g.UpperMass, g.UpperLength);
        var lower = PlanarLink.SlenderBar(Lower, g.LowerMass, g.LowerLength);
        var hand = PlanarLink.SlenderBar(Hand, w.HandMass, w.HandLength);

        var anchor = new Vector2d(-g.ShoulderAnchorBehind, g.ShoulderAnchorAbove);
        var shoulderInsertion = upper.EndA + new Vector2d(g.ShoulderInsertion, 0);
        var elbowPost = upper.EndB + new Vector2d(-g.ElbowPostBehind, g.ElbowPostAbove);
        var elbowInsertion = lower.EndA + new Vector2d(g.ElbowInsertion, 0);
        var wristPost = lower.EndB + new Vector2d(-w.WristPostBehind, w.WristPostAbove);
        var wristInsertion = hand.EndA + new Vector2d(w.WristInsertion, 0);

        // World poses with the shoulder at the origin.
        var upperPose = new Pose2d((-upper.EndA).Rotated(theta[0]), theta[0]);
        var elbow = upperPose.ToWorld(upper.EndB);
        var lowerPose = new Pose2d(elbow + (-lower.EndA).Rotated(theta[1]), theta[1]);
        var wrist = lowerPose.ToWorld(lower.EndB);
        var handPose = new Pose2d(wrist + (-hand.EndA).Rotated(theta[2]), theta[2]);

        double Moment(Vector2d r, double mass) => Vector2d.Cross(r, mass * grav);

        // Wrist: the hand about the wrist, and the preloaded stop pushing it down.
        var wristPostWorld = lowerPose.ToWorld(wristPost);
        var wristInsertionWorld = handPose.ToWorld(wristInsertion);
        var wristTension = Balance(
            Moment(handPose.Position - wrist, w.HandMass) - w.WristPreload,
            wristInsertionWorld - wrist,
            wristPostWorld - wristInsertionWorld,
            WristTendon);

        // Elbow: the lower link and hand about the elbow, and the preloaded
        // stop pushing them down (the wrist tendon and stop are internal to them).
        var elbowPostWorld = upperPose.ToWorld(elbowPost);
        var elbowInsertionWorld = lowerPose.ToWorld(elbowInsertion);
        var elbowTension = Balance(
            Moment(lowerPose.Position - elbow, g.LowerMass) + Moment(handPose.Position - elbow, w.HandMass) - w.ElbowPreload,
            elbowInsertionWorld - elbow,
            elbowPostWorld - elbowInsertionWorld,
            ElbowTendon);

        // Shoulder: all three about the shoulder (the elbow and wrist stops are internal).
        var shoulderInsertionWorld = upperPose.ToWorld(shoulderInsertion);
        var shoulderTension = Balance(
            Moment(upperPose.Position, g.UpperMass) + Moment(lowerPose.Position, g.LowerMass) + Moment(handPose.Position, w.HandMass),
            shoulderInsertionWorld,
            anchor - shoulderInsertionWorld,
            ShoulderTendon);

        Cable Tendon(string id, Attachment a, Attachment b, double length, double tension, double scale) =>
            new(id, a, b, scale * Cable.RestLengthFor(length, tension, g.AxialStiffness), g.AxialStiffness, g.TendonDamping);

        Cable[] cables =
        [
            Tendon(ShoulderTendon, Attachment.Ground(anchor), Attachment.OnLink(Upper, shoulderInsertion), (anchor - shoulderInsertionWorld).Length, shoulderTension, scales[0]),
            Tendon(ElbowTendon, Attachment.OnLink(Upper, elbowPost), Attachment.OnLink(Lower, elbowInsertion), (elbowPostWorld - elbowInsertionWorld).Length, elbowTension, scales[1]),
            Tendon(WristTendon, Attachment.OnLink(Lower, wristPost), Attachment.OnLink(Hand, wristInsertion), (wristPostWorld - wristInsertionWorld).Length, wristTension, scales[2]),
        ];

        // A fold stop at −fold, and a hyperextension stop preload / k short of the design joint angle.
        JointLimit? Limit(double? fold, double design, double preload, double stiffness, double damping)
        {
            var lower = fold is { } f ? -f : double.NegativeInfinity;
            var upperStop = w.HyperextensionStops ? design - (preload / stiffness) : double.PositiveInfinity;
            return double.IsFinite(lower) || double.IsFinite(upperStop) ? new JointLimit(lower, upperStop, stiffness, damping) : null;
        }

        PinJoint[] pins =
        [
            new(Shoulder, Attachment.OnLink(Upper, upper.EndA), Attachment.Ground(Vector2d.Zero)),
            new(Elbow, Attachment.OnLink(Lower, lower.EndA), Attachment.OnLink(Upper, upper.EndB), limit: Limit(g.ElbowFoldStop, theta[1] - theta[0], w.ElbowPreload, g.ElbowStopStiffness, g.ElbowStopDamping)),
            new(Wrist, Attachment.OnLink(Hand, hand.EndA), Attachment.OnLink(Lower, lower.EndB), limit: Limit(w.WristFoldStop, theta[2] - theta[1], w.WristPreload, w.WristStopStiffness, w.WristStopDamping)),
        ];

        var system = new PlanarSystem([upper, lower, hand], grav, pins, cables, loads: loads);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, upperPose, default);
        system.SetBody(state, 1, lowerPose, default);
        system.SetBody(state, 2, handPose, default);
        return new PlanarModel("locking_wing", system, state);
    }

    /// <summary>The tension along <paramref name="direction"/> at <paramref name="r"/> whose moment cancels <paramref name="load"/>.</summary>
    private static double Balance(double load, Vector2d r, Vector2d direction, string tendon)
    {
        var tension = -load / Vector2d.Cross(r, direction / direction.Length);
        if (!(tension > 0) || !double.IsFinite(tension))
        {
            throw new ArgumentException($"The {tendon} cannot hold this pose: it would have to push.");
        }

        return tension;
    }
}
