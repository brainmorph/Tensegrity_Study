using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Output;

/// <summary>
/// Maps a <see cref="TopologyDocument"/> to a <see cref="PlanarSystem"/> and
/// its initial state, and back. Both directions copy every number as is, so
/// a system that goes to a document and back is bit-for-bit the same.
/// </summary>
public static class TopologyMapping
{
    /// <summary>Builds the system and its initial state (dissipated energy zero) from a document.</summary>
    public static (PlanarSystem System, double[] State) FromDocument(TopologyDocument document)
    {
        ArgumentNullException.ThrowIfNull(document);
        var where = $"topology '{document.Name}'";
        if (document.FormatVersion != TopologyDocument.CurrentFormatVersion)
        {
            throw new FormatException($"{where} has format version {document.FormatVersion}; expected {TopologyDocument.CurrentFormatVersion}.");
        }

        var links = document.Links.Select(l => new PlanarLink(l.Id, l.Mass, l.Inertia, l.Length, ParseRole(l.Role, where))).ToArray();
        var pins = document.Pins.Select(p => new PinJoint(p.Id, ToAttachment(p.A, where), ToAttachment(p.B, where))).ToArray();
        var cables = document.Cables.Select(c =>
            new Cable(c.Id, ToAttachment(c.A, where), ToAttachment(c.B, where), c.RestLength, c.AxialStiffness, c.Damping)).ToArray();

        var system = new PlanarSystem(
            links,
            ToVector(document.Gravity, $"{where} gravity"),
            pins,
            cables,
            new ConstraintStabilization(document.ConstraintStabilization));

        var state = system.CreateState().Values;
        for (var b = 0; b < links.Length; b++)
        {
            var l = document.Links[b];
            var pose = l.Pose ?? throw new FormatException($"{where}: link '{l.Id}' has no pose.");
            var twist = l.Twist ?? new TwistDocument();
            system.SetBody(state, b, new Pose2d(new Vector2d(pose.X, pose.Y), pose.Theta), new Twist2d(new Vector2d(twist.Vx, twist.Vy), twist.Omega));
        }

        return (system, state);
    }

    /// <summary>Describes a system with the link poses and velocities of <paramref name="state"/>.</summary>
    public static TopologyDocument ToDocument(
        PlanarSystem system,
        ReadOnlySpan<double> state,
        string name,
        string? description = null,
        SimulationDocument? simulation = null)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentException.ThrowIfNullOrWhiteSpace(name);

        var links = new List<LinkDocument>(system.BodyCount);
        for (var b = 0; b < system.BodyCount; b++)
        {
            var link = system.Links[b];
            var pose = system.GetPose(state, b);
            var twist = system.GetTwist(state, b);
            links.Add(new LinkDocument
            {
                Id = link.Id,
                Role = link.Role == LinkRole.Strut ? LinkRoles.Strut : LinkRoles.Link,
                Mass = link.Mass,
                Inertia = link.MomentOfInertia,
                Length = link.Length,
                Pose = new PoseDocument { X = pose.Position.X, Y = pose.Position.Y, Theta = pose.Angle },
                Twist = new TwistDocument { Vx = twist.Linear.X, Vy = twist.Linear.Y, Omega = twist.Angular },
            });
        }

        return new TopologyDocument
        {
            Name = name,
            Description = description,
            Gravity = [system.Gravity.X, system.Gravity.Y],
            ConstraintStabilization = system.Stabilization.Frequency,
            Links = links,
            Pins = system.Pins.Select(p => new PinDocument { Id = p.Id, A = ToDocument(p.A), B = ToDocument(p.B) }).ToList(),
            Cables = system.Cables.Select(c => new CableDocument
            {
                Id = c.Id,
                A = ToDocument(c.A),
                B = ToDocument(c.B),
                RestLength = c.RestLength,
                AxialStiffness = c.AxialStiffness,
                Damping = c.Damping,
            }).ToList(),
            Simulation = simulation,
        };
    }

    private static AttachmentDocument ToDocument(Attachment attachment) =>
        attachment.IsGround
            ? AttachmentDocument.AtGround(attachment.Point.X, attachment.Point.Y)
            : AttachmentDocument.OnLink(attachment.LinkId!, attachment.Point.X, attachment.Point.Y);

    private static Attachment ToAttachment(AttachmentDocument? document, string where)
    {
        if (document is null)
        {
            throw new FormatException($"{where}: missing attachment.");
        }

        var onLink = document.Link is not null || document.Point is not null;
        if (onLink == document.IsGround)
        {
            throw new FormatException($"{where}: an attachment needs either 'link' with 'point', or 'ground', not both or neither.");
        }

        if (document.IsGround)
        {
            return Attachment.Ground(ToVector(document.Ground, $"{where} ground point"));
        }

        if (document.Link is null)
        {
            throw new FormatException($"{where}: attachment has a 'point' but no 'link'.");
        }

        return Attachment.OnLink(document.Link, ToVector(document.Point, $"{where} point on '{document.Link}'"));
    }

    private static Vector2d ToVector(double[]? values, string what) =>
        values is { Length: 2 } ? new Vector2d(values[0], values[1]) : throw new FormatException($"{what} must be [x, y].");

    private static LinkRole ParseRole(string? role, string where) => role switch
    {
        LinkRoles.Link or null => LinkRole.Link,
        LinkRoles.Strut => LinkRole.Strut,
        _ => throw new FormatException($"{where}: unknown link role '{role}'; expected '{LinkRoles.Link}' or '{LinkRoles.Strut}'."),
    };
}
