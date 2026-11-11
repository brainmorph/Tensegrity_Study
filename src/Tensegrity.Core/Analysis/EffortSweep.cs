using System.Text.Json;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Analysis;

/// <summary>One cable setting in an <see cref="EffortSweep"/>.</summary>
/// <param name="Scale">Rest length as a multiple of the topology's own.</param>
/// <param name="RestLength">The cable's rest length at this scale, m.</param>
/// <param name="Tension">The cable's tension in the topology's initial pose at rest, N.</param>
/// <param name="Torques">Holding torque at each swept pin, N·m (+ on the pin's end A).</param>
/// <param name="MaxLinearAcceleration">Acceleration the pins cannot cancel, m/s²; zero when they hold the pose.</param>
/// <param name="MaxAngularAcceleration">Angular acceleration the pins cannot cancel, rad/s².</param>
/// <param name="Redundancy">Torque combinations at the pins that move nothing; above zero the torques are the minimum-norm set.</param>
public sealed record EffortPoint(
    double Scale,
    double RestLength,
    double Tension,
    IReadOnlyList<double> Torques,
    double MaxLinearAcceleration,
    double MaxAngularAcceleration,
    int Redundancy = 0);

/// <summary>One bend of a joint in an <see cref="EffortSweep.Bend"/> sweep.</summary>
/// <param name="Offset">How far the joint is bent from the topology's pose, rad.</param>
/// <param name="JointAngle">The joint angle φ = θ_A − θ_B after the bend, rad.</param>
/// <param name="Tensions">Every cable's tension in the bent pose at rest, N, in the topology's cable order.</param>
/// <param name="Torques">Holding torque at each swept pin, N·m (+ on the pin's end A).</param>
/// <param name="MaxLinearAcceleration">Acceleration the pins cannot cancel, m/s²; zero when they hold the pose.</param>
/// <param name="MaxAngularAcceleration">Angular acceleration the pins cannot cancel, rad/s².</param>
/// <param name="Redundancy">Torque combinations at the pins that move nothing; above zero the torques are the minimum-norm set.</param>
public sealed record BendPoint(
    double Offset,
    double JointAngle,
    IReadOnlyList<double> Tensions,
    IReadOnlyList<double> Torques,
    double MaxLinearAcceleration,
    double MaxAngularAcceleration,
    int Redundancy = 0);

/// <summary>
/// Effort curves (PLAN.md M5, M6): the holding torque needed to keep a
/// pose still (test 11's measure). <see cref="Run"/> keeps the topology's
/// initial pose and scales one cable's rest length; zero torque means the
/// cables hold the pose by themselves, and the slope shows how much effort
/// a mis-set tendon costs. The pose itself never changes, so a lengthened
/// cable that goes slack shows the full load falling on the joints.
/// <see cref="Bend"/> keeps the cables and bends one joint instead, which
/// shows how hard the structure pushes back when a joint is moved off the
/// pose: the stiffness of a tendon lock, and where its tendons go slack.
///
/// Poses are taken at rest: any initial velocities in the topology are
/// dropped, so no damping or velocity terms enter.
/// </summary>
public static class EffortSweep
{
    public static IReadOnlyList<EffortPoint> Run(
        TopologyDocument document,
        string cableId,
        IReadOnlyList<string> pinIds,
        IEnumerable<double> scales)
    {
        ArgumentNullException.ThrowIfNull(document);
        ArgumentNullException.ThrowIfNull(pinIds);
        ArgumentNullException.ThrowIfNull(scales);
        var index = document.Cables.FindIndex(c => string.Equals(c.Id, cableId, StringComparison.Ordinal));
        if (index < 0)
        {
            throw new ArgumentException($"Topology '{document.Name}' has no cable '{cableId}'.", nameof(cableId));
        }

        var baseRest = document.Cables[index].RestLength;
        var points = new List<EffortPoint>();
        foreach (var scale in scales)
        {
            var copy = AtRest(document);
            copy.Cables[index].RestLength = scale * baseRest;
            var (tensions, hold) = Solve(copy, pinIds);
            points.Add(new EffortPoint(scale, scale * baseRest, tensions[index], hold.Torques, hold.MaxLinearAcceleration, hold.MaxAngularAcceleration, hold.Redundancy));
        }

        return points;
    }

    /// <summary>
    /// Holding torques at <paramref name="pinIds"/> as pin
    /// <paramref name="bendPinId"/> is bent by each of
    /// <paramref name="offsets"/> (rad) from the topology's pose. A bend of
    /// Δ turns the links on the pin's end-A side rigidly about the pin by
    /// +Δ, so the joint angle φ = θ_A − θ_B grows by Δ; if end A's side is
    /// tied to the ground some other way, end B's side turns by −Δ instead.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// The pin does not exist, or it is in a closed loop: both its sides
    /// reach the ground, or each other, through other pins, so it cannot be
    /// bent alone.
    /// </exception>
    public static IReadOnlyList<BendPoint> Bend(
        TopologyDocument document,
        string bendPinId,
        IReadOnlyList<string> pinIds,
        IEnumerable<double> offsets)
    {
        ArgumentNullException.ThrowIfNull(document);
        ArgumentNullException.ThrowIfNull(pinIds);
        ArgumentNullException.ThrowIfNull(offsets);
        var pin = document.Pins.Find(p => string.Equals(p.Id, bendPinId, StringComparison.Ordinal))
            ?? throw new ArgumentException($"Topology '{document.Name}' has no pin '{bendPinId}'.", nameof(bendPinId));
        var (side, sign) = FreeSide(document, pin);
        var baseAngle = Angle(document, pin.A) - Angle(document, pin.B);

        var points = new List<BendPoint>();
        foreach (var offset in offsets)
        {
            var copy = AtRest(document);
            var turn = sign * offset;
            var center = WorldPoint(copy, sign > 0 ? pin.A : pin.B);
            foreach (var link in copy.Links.Where(l => side.Contains(l.Id)))
            {
                var p = link.Pose;
                var (sin, cos) = Math.SinCos(turn);
                double dx = p.X - center.X, dy = p.Y - center.Y;
                link.Pose = new PoseDocument { X = center.X + (cos * dx) - (sin * dy), Y = center.Y + (sin * dx) + (cos * dy), Theta = p.Theta + turn };
            }

            var (tensions, hold) = Solve(copy, pinIds);
            points.Add(new BendPoint(offset, baseAngle + offset, tensions, hold.Torques, hold.MaxLinearAcceleration, hold.MaxAngularAcceleration, hold.Redundancy));
        }

        return points;
    }

    /// <summary>An independent copy of the document, so the caller's document is untouched.</summary>
    internal static TopologyDocument Copy(TopologyDocument document) =>
        JsonSerializer.Deserialize<TopologyDocument>(RunFiles.SerializeTopology(document), RunFiles.Json)!;

    /// <summary>An independent copy of the document with every link at rest.</summary>
    internal static TopologyDocument AtRest(TopologyDocument document)
    {
        var copy = Copy(document);
        foreach (var link in copy.Links)
        {
            link.Twist = new TwistDocument();
        }

        return copy;
    }

    /// <summary>Every cable's tension, and the holding torques at <paramref name="pinIds"/>, in the document's initial state.</summary>
    internal static (double[] Tensions, HoldingTorques Hold) Solve(TopologyDocument document, IReadOnlyList<string> pinIds)
    {
        var (system, state) = TopologyMapping.FromDocument(document);
        var tensions = new double[system.Cables.Count];
        system.GetCableTensions(state, tensions);
        return (tensions, system.SolveHoldingTorques(0, state, pinIds));
    }

    /// <summary>
    /// The links that turn when <paramref name="pin"/> bends, and the sign of
    /// their turn: end A's side (+1) when it reaches neither the ground nor
    /// end B through the other pins, else end B's side (−1) on the same terms.
    /// </summary>
    private static (HashSet<string> Side, int Sign) FreeSide(TopologyDocument document, PinDocument pin)
    {
        foreach (var (end, other, sign) in new[] { (pin.A, pin.B, 1), (pin.B, pin.A, -1) })
        {
            if (end.IsGround)
            {
                continue;
            }

            var side = Reachable(document, pin, end.Link!, out var touchesGround);
            if (!touchesGround && (other.IsGround || !side.Contains(other.Link!)))
            {
                return (side, sign);
            }
        }

        throw new ArgumentException(
            $"Pin '{pin.Id}' of '{document.Name}' is in a closed loop: neither side can turn about it alone.", nameof(pin));
    }

    /// <summary>Links joined to <paramref name="start"/> by pins other than <paramref name="cut"/>, and whether any of those pins is to the ground.</summary>
    private static HashSet<string> Reachable(TopologyDocument document, PinDocument cut, string start, out bool touchesGround)
    {
        var seen = new HashSet<string>(StringComparer.Ordinal) { start };
        var queue = new Queue<string>([start]);
        touchesGround = false;
        while (queue.Count > 0)
        {
            var link = queue.Dequeue();
            foreach (var pin in document.Pins.Where(p => !ReferenceEquals(p, cut)))
            {
                var (here, there) = pin.A.Link == link ? (pin.A, pin.B) : pin.B.Link == link ? (pin.B, pin.A) : (null, null);
                if (here is null)
                {
                    continue;
                }

                if (there!.IsGround)
                {
                    touchesGround = true;
                }
                else if (seen.Add(there.Link!))
                {
                    queue.Enqueue(there.Link!);
                }
            }
        }

        return seen;
    }

    private static double Angle(TopologyDocument document, AttachmentDocument end) =>
        end.IsGround ? 0 : Link(document, end.Link!).Pose.Theta;

    /// <summary>Where an attachment is in the document's initial pose, world frame.</summary>
    internal static (double X, double Y) WorldPoint(TopologyDocument document, AttachmentDocument end)
    {
        if (end.IsGround)
        {
            return (end.Ground![0], end.Ground[1]);
        }

        var pose = Link(document, end.Link!).Pose;
        var (sin, cos) = Math.SinCos(pose.Theta);
        double x = end.Point![0], y = end.Point[1];
        return (pose.X + (cos * x) - (sin * y), pose.Y + (sin * x) + (cos * y));
    }

    private static LinkDocument Link(TopologyDocument document, string id) =>
        document.Links.Find(l => string.Equals(l.Id, id, StringComparison.Ordinal))
            ?? throw new ArgumentException($"Topology '{document.Name}' has no link '{id}'.", nameof(id));
}
