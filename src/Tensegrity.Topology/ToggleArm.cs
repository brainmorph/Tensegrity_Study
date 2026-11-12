using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// Dimensions of a <see cref="ToggleArm"/>, SI units. The links and the
/// shoulder tendon match <see cref="LockingArmGeometry"/>'s defaults; the
/// elbow is new. The elbow tendon's ends were chosen by a quasi-static scan
/// (PLAN.md M8) for a short-travel toggle whose peak strain stays under 8%.
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
/// <param name="ElbowInsertion">Where the elbow tendon attaches along the lower link, measured from the elbow, m.</param>
/// <param name="ElbowInsertionAbove">How far that attachment sits above the lower link's axis, m.</param>
/// <param name="ElbowTension">Elbow tendon tension with the arm straight, N. It sets the tendon's rest length.</param>
/// <param name="ElbowFoldStop">How far the elbow may fold downward from straight before its stop engages, rad: the folded pole.</param>
/// <param name="ElbowStopStiffness">Torsional stiffness of both elbow stops, N·m/rad.</param>
/// <param name="ElbowStopDamping">Torsional damping of both elbow stops, N·m·s/rad.</param>
/// <param name="AxialStiffness">Tendon EA, N.</param>
/// <param name="TendonDamping">Tendon viscous damping c, N·s/m.</param>
public sealed record ToggleArmGeometry(
    double UpperLength = 0.30,
    double UpperMass = 0.20,
    double LowerLength = 0.35,
    double LowerMass = 0.12,
    double ShoulderAnchorBehind = 0.03,
    double ShoulderAnchorAbove = 0.05,
    double ShoulderInsertion = 0.12,
    double ElbowPostBehind = 0.05,
    double ElbowPostAbove = 0.03,
    double ElbowInsertion = 0.15,
    double ElbowInsertionAbove = 0.03,
    double ElbowTension = 20.0,
    double ElbowFoldStop = Math.PI / 3,
    double ElbowStopStiffness = 20.0,
    double ElbowStopDamping = 0.2,
    double AxialStiffness = 1000.0,
    double TendonDamping = 150.0);

/// <summary>
/// The two-link toggle arm (PLAN.md M8): a snap latch at the elbow. The
/// links and shoulder lock are the <see cref="LockingArm"/>'s, but the elbow
/// tendon runs <b>over center</b>.
/// <code>
///   anchor ●           post ●━━━━━━━━━━━━━━━━━━━● insertion
///           ╲ shoulder                elbow tendon
///   shoulder ○━━━━━━ upper ━━━━━━━○━━━━━━━ lower ━━━━━━━━
///                               elbow
/// </code>
/// The elbow tendon runs from a post behind and above the elbow to a point
/// above the lower link, further out. Its length is greatest at the elbow
/// angle φ* where its line passes through the pin. On either side of φ*,
/// shortening the tendon turns the joint away from φ*, so the tendon pulls
/// the elbow into whichever stop is on its side: the stop at straight (the
/// extended pole) or the fold stop (the folded pole). Both poles are held
/// with zero torque, and passing from one to the other means stretching the
/// tendon over φ*: an energy barrier.
///
/// The elbow tendon is tensioned to <see cref="ToggleArmGeometry.ElbowTension"/>
/// straight out, which presses the elbow into its straight stop with the
/// torque its moment leaves over after carrying the lower link's weight. A
/// penalty stop pushes only once the joint is past it, so the straight stop
/// sits that torque / k short of straight, which makes straight out an exact
/// equilibrium. The shoulder tendon is tensioned to hold both links' weight,
/// as in the locking arm; the elbow tendon and stops are internal to the
/// arm.
///
/// The shoulder is at the origin. Pin ends A are the outer links, so a
/// positive holding torque lifts the arm, and the elbow angle is
/// θ_lower − θ_upper.
/// </summary>
public static class ToggleArm
{
    public const string Upper = LockingArm.Upper;
    public const string Lower = LockingArm.Lower;
    public const string Shoulder = LockingArm.Shoulder;
    public const string Elbow = LockingArm.Elbow;
    public const string ShoulderTendon = LockingArm.ShoulderTendon;
    public const string ElbowTendon = LockingArm.ElbowTendon;

    /// <summary>
    /// The torque the elbow tendon and the lower link's weight leave pressing
    /// the straight elbow up into its stop, N·m. Positive means the
    /// straight pose is latched.
    /// </summary>
    public static double StraightPress(ToggleArmGeometry? geometry = null, Vector2d? gravity = null)
    {
        var g = geometry ?? new ToggleArmGeometry();
        var grav = gravity ?? PlanarSystem.StandardGravity;
        var post = new Vector2d(-g.ElbowPostBehind, g.ElbowPostAbove);
        var insertion = new Vector2d(g.ElbowInsertion, g.ElbowInsertionAbove);
        var direction = (post - insertion) / (post - insertion).Length;
        return Vector2d.Cross(insertion, g.ElbowTension * direction)
            + Vector2d.Cross(new Vector2d(g.LowerLength / 2, 0), g.LowerMass * grav);
    }

    /// <summary>Id of the elbow twitch actuator.</summary>
    public const string ElbowTwitch = "elbow_twitch";

    /// <summary>
    /// A brief muscle twitch across the elbow: a triangular torque pulse of
    /// angular impulse <paramref name="impulse"/> N·m·s, starting at
    /// <paramref name="start"/> and lasting <paramref name="duration"/>. A
    /// negative impulse folds the elbow, a positive one extends it.
    /// </summary>
    public static JointActuator Twitch(double impulse, double start, double duration) =>
        Twitches([(impulse, start)], duration);

    /// <summary>
    /// Several twitches of the same elbow muscle, one actuator: a triangular
    /// torque pulse of <paramref name="duration"/> per entry, each of its
    /// angular impulse, N·m·s, at its start time. A pin takes one actuator,
    /// so a sequence of twitches is one schedule. Pulses must not overlap.
    /// </summary>
    public static JointActuator Twitches(IReadOnlyList<(double Impulse, double Start)> pulses, double duration)
    {
        if (pulses.Count == 0 || !(duration > 0) || !double.IsFinite(duration))
        {
            throw new ArgumentException("Twitches need at least one pulse and a positive duration.");
        }

        var points = new List<SchedulePoint>();
        var free = 0.0;
        foreach (var (impulse, start) in pulses)
        {
            if (!double.IsFinite(impulse) || !(start >= free) || (points.Count > 0 && !(start > free)))
            {
                throw new ArgumentException("Each twitch needs a finite impulse and must start at or after 0 and after the one before it ends.", nameof(pulses));
            }

            points.Add(new SchedulePoint(start, 0));
            points.Add(new SchedulePoint(start + (duration / 2), 2 * impulse / duration));
            points.Add(new SchedulePoint(start + duration, 0));
            free = start + duration;
        }

        return new JointActuator(ElbowTwitch, Elbow, new Schedule(points));
    }

    /// <summary>
    /// The toggle arm at rest straight out, latched on its straight stop.
    /// Optional <paramref name="actuators"/> (such as a <see cref="Twitch"/>)
    /// and <paramref name="loads"/> perturb it.
    /// </summary>
    public static PlanarModel Create(
        ToggleArmGeometry? geometry = null,
        Vector2d? gravity = null,
        IEnumerable<JointActuator>? actuators = null,
        IEnumerable<ExternalLoad>? loads = null)
    {
        var g = geometry ?? new ToggleArmGeometry();
        var grav = gravity ?? PlanarSystem.StandardGravity;
        RequireInside(g.ShoulderInsertion, g.UpperLength, nameof(g.ShoulderInsertion));
        RequireInside(g.ElbowInsertion, g.LowerLength, nameof(g.ElbowInsertion));
        if (!(g.ElbowTension > 0) || !double.IsFinite(g.ElbowTension))
        {
            throw new ArgumentOutOfRangeException(nameof(geometry), g.ElbowTension, "The elbow tension must be finite and positive.");
        }

        if (!(g.ElbowFoldStop > 0 && g.ElbowFoldStop < Math.PI))
        {
            throw new ArgumentOutOfRangeException(nameof(geometry), g.ElbowFoldStop, "The fold stop must lie in (0, π).");
        }

        var press = StraightPress(g, grav);
        if (!(press > 0))
        {
            throw new ArgumentException($"The elbow tendon does not latch the straight pose: it leaves {press} N m pressing the elbow up.", nameof(geometry));
        }

        var upper = PlanarLink.SlenderBar(Upper, g.UpperMass, g.UpperLength);
        var lower = PlanarLink.SlenderBar(Lower, g.LowerMass, g.LowerLength);

        // Straight out along +x, the shoulder at the origin.
        var upperPose = new Pose2d(-upper.EndA, 0);
        var elbow = upperPose.ToWorld(upper.EndB);
        var lowerPose = new Pose2d(elbow - lower.EndA, 0);

        var anchor = new Vector2d(-g.ShoulderAnchorBehind, g.ShoulderAnchorAbove);
        var shoulderInsertion = upper.EndA + new Vector2d(g.ShoulderInsertion, 0);
        var post = upper.EndB + new Vector2d(-g.ElbowPostBehind, g.ElbowPostAbove);
        var elbowInsertion = lower.EndA + new Vector2d(g.ElbowInsertion, g.ElbowInsertionAbove);

        // Shoulder: both links' weight about the shoulder (the elbow tendon and stops are internal).
        var shoulderInsertionWorld = upperPose.ToWorld(shoulderInsertion);
        var shoulderSpan = anchor - shoulderInsertionWorld;
        var load = Vector2d.Cross(upperPose.Position, g.UpperMass * grav) + Vector2d.Cross(lowerPose.Position, g.LowerMass * grav);
        var shoulderTension = -load / Vector2d.Cross(shoulderInsertionWorld, shoulderSpan / shoulderSpan.Length);
        if (!(shoulderTension > 0) || !double.IsFinite(shoulderTension))
        {
            throw new ArgumentException("The shoulder tendon cannot hold the arm straight out: it would have to push.", nameof(geometry));
        }

        var elbowLength = (upperPose.ToWorld(post) - lowerPose.ToWorld(elbowInsertion)).Length;
        Cable[] cables =
        [
            new(
                ShoulderTendon,
                Attachment.Ground(anchor),
                Attachment.OnLink(Upper, shoulderInsertion),
                Cable.RestLengthFor(shoulderSpan.Length, shoulderTension, g.AxialStiffness),
                g.AxialStiffness,
                g.TendonDamping),
            new(
                ElbowTendon,
                Attachment.OnLink(Upper, post),
                Attachment.OnLink(Lower, elbowInsertion),
                Cable.RestLengthFor(elbowLength, g.ElbowTension, g.AxialStiffness),
                g.AxialStiffness,
                g.TendonDamping),
        ];

        PinJoint[] pins =
        [
            new(Shoulder, Attachment.OnLink(Upper, upper.EndA), Attachment.Ground(Vector2d.Zero)),
            new(
                Elbow,
                Attachment.OnLink(Lower, lower.EndA),
                Attachment.OnLink(Upper, upper.EndB),
                limit: new JointLimit(-g.ElbowFoldStop, -press / g.ElbowStopStiffness, g.ElbowStopStiffness, g.ElbowStopDamping)),
        ];

        var system = new PlanarSystem([upper, lower], grav, pins, cables, actuators: actuators, loads: loads);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, upperPose, default);
        system.SetBody(state, 1, lowerPose, default);
        return new PlanarModel("toggle_arm", system, state);
    }

    private static void RequireInside(double value, double length, string name)
    {
        if (!double.IsFinite(value) || value <= 0 || value > length)
        {
            throw new ArgumentOutOfRangeException(name, value, $"Must lie on the link, in (0, {length}].");
        }
    }
}
