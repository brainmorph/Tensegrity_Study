using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// Dimensions of a <see cref="LockingArm"/>, SI units. The defaults are
/// generic, roughly bird-wing scale (PLAN.md 2: no specific bird).
/// </summary>
/// <param name="UpperLength">Upper link (shoulder to elbow), m.</param>
/// <param name="UpperMass">Upper link mass, kg.</param>
/// <param name="LowerLength">Lower link (elbow outward), m.</param>
/// <param name="LowerMass">Lower link mass, kg.</param>
/// <param name="ShoulderAnchorBehind">How far the shoulder tendon's body anchor sits behind the shoulder (−x), m.</param>
/// <param name="ShoulderAnchorAbove">How far the anchor sits above the shoulder (+y), m.</param>
/// <param name="ShoulderInsertion">Where the shoulder tendon attaches to the upper link, measured from the shoulder, m.</param>
/// <param name="ElbowPostBehind">How far the elbow tendon's post sits behind the elbow, along the upper link, m.</param>
/// <param name="ElbowPostAbove">How far the post sits above the upper link's axis, m.</param>
/// <param name="ElbowInsertion">Where the elbow tendon attaches to the lower link, measured from the elbow, m.</param>
/// <param name="AxialStiffness">Tendon EA, N.</param>
/// <param name="TendonDamping">Tendon viscous damping c, N·s/m.</param>
public sealed record LockingArmGeometry(
    double UpperLength = 0.30,
    double UpperMass = 0.20,
    double LowerLength = 0.35,
    double LowerMass = 0.12,
    double ShoulderAnchorBehind = 0.03,
    double ShoulderAnchorAbove = 0.05,
    double ShoulderInsertion = 0.12,
    double ElbowPostBehind = 0.03,
    double ElbowPostAbove = 0.04,
    double ElbowInsertion = 0.10,
    double AxialStiffness = 1000.0,
    double TendonDamping = 150.0);

/// <summary>
/// The two-link locking arm (PLAN.md section 8, M5): an upper link pinned
/// to the body at the shoulder and a lower link pinned to it at the elbow,
/// each joint held up against gravity by one tendon and no muscle torque.
/// <code>
///   anchor ●                post ●
///           ╲ shoulder          ╲ elbow tendon
///            ╲ tendon            ╲
///   shoulder ○━━━━━━ upper ━━━━━━━○━━━━━━━ lower ━━━━━━━━
///                                elbow
/// </code>
/// Each tendon runs from a point behind and above its joint (a body anchor
/// for the shoulder, a post on the upper link for the elbow) to a point
/// further out on the next link. Gravity pulls the arm down and the
/// stretched tendon pulls it back, so the pose is a stable equilibrium at
/// zero torque: the joint is locked (PLAN.md 7, test 10). Because the
/// tendon starts behind the joint, its line of pull swings toward the joint
/// as the link droops and its leverage falls. Lengthen the tendon far
/// enough and the stable pose disappears (a fold): the arm collapses.
///
/// The shoulder is at the origin. Pin ends A are the outer links (upper at
/// the shoulder, lower at the elbow), so a positive holding torque lifts the
/// arm.
///
/// Tendon tensions come from moment balance at the chosen pose, worked
/// backwards to rest lengths with <see cref="Cable.RestLengthFor"/>, so the
/// pose is an exact equilibrium. The elbow tendon is internal to the arm,
/// so the shoulder tendon carries the gravity moment of both links about
/// the shoulder and the elbow tendon that of the lower link about the
/// elbow.
/// </summary>
public static class LockingArm
{
    public const string Upper = "upper";
    public const string Lower = "lower";
    public const string Shoulder = "shoulder";
    public const string Elbow = "elbow";
    public const string ShoulderTendon = "shoulder_tendon";
    public const string ElbowTendon = "elbow_tendon";

    /// <summary>
    /// The arm at rest with the upper link at world angle
    /// <paramref name="upperAngle"/> and the lower at
    /// <paramref name="lowerAngle"/> (0 is straight out along +x). The
    /// tendons are tensioned to hold that pose, then their rest lengths are
    /// multiplied by the given scales: 1 keeps the exact equilibrium, more
    /// than 1 lengthens a tendon.
    /// </summary>
    public static PlanarModel Create(
        LockingArmGeometry? geometry = null,
        double upperAngle = 0,
        double lowerAngle = 0,
        double shoulderRestLengthScale = 1,
        double elbowRestLengthScale = 1,
        Vector2d? gravity = null)
    {
        var g = geometry ?? new LockingArmGeometry();
        var grav = gravity ?? PlanarSystem.StandardGravity;
        RequirePositive(shoulderRestLengthScale, nameof(shoulderRestLengthScale));
        RequirePositive(elbowRestLengthScale, nameof(elbowRestLengthScale));
        RequireInside(g.ShoulderInsertion, g.UpperLength, nameof(g.ShoulderInsertion));
        RequireInside(g.ElbowInsertion, g.LowerLength, nameof(g.ElbowInsertion));

        var upper = PlanarLink.SlenderBar(Upper, g.UpperMass, g.UpperLength);
        var lower = PlanarLink.SlenderBar(Lower, g.LowerMass, g.LowerLength);

        var anchor = new Vector2d(-g.ShoulderAnchorBehind, g.ShoulderAnchorAbove);
        var shoulderInsertion = upper.EndA + new Vector2d(g.ShoulderInsertion, 0);
        var post = upper.EndB + new Vector2d(-g.ElbowPostBehind, g.ElbowPostAbove);
        var elbowInsertion = lower.EndA + new Vector2d(g.ElbowInsertion, 0);

        // World pose with the shoulder at the origin.
        var upperPose = new Pose2d((-upper.EndA).Rotated(upperAngle), upperAngle);
        var elbow = upperPose.ToWorld(upper.EndB);
        var lowerPose = new Pose2d(elbow + (-lower.EndA).Rotated(lowerAngle), lowerAngle);

        // Elbow: the tendon's moment on the lower link cancels the lower link's weight about the elbow.
        var postWorld = upperPose.ToWorld(post);
        var elbowInsertionWorld = lowerPose.ToWorld(elbowInsertion);
        var elbowTension = Balance(
            Moment(lowerPose.Position - elbow, g.LowerMass * grav),
            elbowInsertionWorld - elbow,
            postWorld - elbowInsertionWorld,
            ElbowTendon);

        // Shoulder: the tendon cancels both links' weight about the shoulder.
        var shoulderInsertionWorld = upperPose.ToWorld(shoulderInsertion);
        var shoulderTension = Balance(
            Moment(upperPose.Position, g.UpperMass * grav) + Moment(lowerPose.Position, g.LowerMass * grav),
            shoulderInsertionWorld,
            anchor - shoulderInsertionWorld,
            ShoulderTendon);

        var shoulderLength = (anchor - shoulderInsertionWorld).Length;
        var elbowLength = (postWorld - elbowInsertionWorld).Length;
        Cable[] cables =
        [
            new(
                ShoulderTendon,
                Attachment.Ground(anchor),
                Attachment.OnLink(Upper, shoulderInsertion),
                shoulderRestLengthScale * Cable.RestLengthFor(shoulderLength, shoulderTension, g.AxialStiffness),
                g.AxialStiffness,
                g.TendonDamping),
            new(
                ElbowTendon,
                Attachment.OnLink(Upper, post),
                Attachment.OnLink(Lower, elbowInsertion),
                elbowRestLengthScale * Cable.RestLengthFor(elbowLength, elbowTension, g.AxialStiffness),
                g.AxialStiffness,
                g.TendonDamping),
        ];

        PinJoint[] pins =
        [
            new(Shoulder, Attachment.OnLink(Upper, upper.EndA), Attachment.Ground(Vector2d.Zero)),
            new(Elbow, Attachment.OnLink(Lower, lower.EndA), Attachment.OnLink(Upper, upper.EndB)),
        ];

        var system = new PlanarSystem([upper, lower], grav, pins, cables);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, upperPose, default);
        system.SetBody(state, 1, lowerPose, default);
        return new PlanarModel("locking_arm", system, state);
    }

    /// <summary>Moment about the origin of a force at <paramref name="r"/>, N·m.</summary>
    private static double Moment(Vector2d r, Vector2d force) => Vector2d.Cross(r, force);

    /// <summary>The tension along <paramref name="direction"/> at <paramref name="r"/> whose moment cancels <paramref name="load"/>.</summary>
    private static double Balance(double load, Vector2d r, Vector2d direction, string tendon)
    {
        var perUnitTension = Vector2d.Cross(r, direction / direction.Length);
        var tension = -load / perUnitTension;
        if (!(tension > 0) || !double.IsFinite(tension))
        {
            throw new ArgumentException($"The {tendon} cannot hold this pose: it would have to push.");
        }

        return tension;
    }

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }

    private static void RequireInside(double value, double length, string name)
    {
        if (!double.IsFinite(value) || value <= 0 || value > length)
        {
            throw new ArgumentOutOfRangeException(name, value, $"Must lie on the link, in (0, {length}].");
        }
    }
}
