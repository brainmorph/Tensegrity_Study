using System.Text.Json.Serialization;

namespace Tensegrity.RunFormat;

/// <summary>
/// A planar topology as plain data: the hand-authored JSON files in
/// <c>topologies/</c> (PLAN.md 9.7) and the <c>topology</c> section of a run
/// manifest (PLAN.md 6) use this same shape. No physics: the core maps it
/// to and from its simulation types. Units are SI (m, kg, s, rad, N).
/// </summary>
public sealed class TopologyDocument
{
    public const int CurrentFormatVersion = 1;

    public int FormatVersion { get; set; } = CurrentFormatVersion;

    public string Name { get; set; } = string.Empty;

    public string? Description { get; set; }

    /// <summary>Uniform gravity [gx, gy], m/s².</summary>
    public double[] Gravity { get; set; } = [0, -9.81];

    /// <summary>Baumgarte natural frequency for pin constraints, rad/s; 0 turns it off.</summary>
    public double ConstraintStabilization { get; set; } = 50;

    /// <summary>
    /// Global settling damping γ, 1/s: each body feels −γ·m·v and −γ·I·ω.
    /// For settling experiments only (PLAN.md 4.3); omitted from the file
    /// when zero, as in every realistic run.
    /// </summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
    public double GlobalDamping { get; set; }

    public List<LinkDocument> Links { get; set; } = [];

    public List<PinDocument> Pins { get; set; } = [];

    public List<CableDocument> Cables { get; set; } = [];

    /// <summary>Joint actuators; null (omitted from the file) when there are none.</summary>
    public List<ActuatorDocument>? Actuators { get; set; }

    /// <summary>External loads (perturbations); null (omitted from the file) when there are none.</summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public List<LoadDocument>? Loads { get; set; }

    /// <summary>The ground line; null (omitted from the file) when there is none.</summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public GroundDocument? Ground { get; set; }

    /// <summary>Default run settings; command-line flags may override them.</summary>
    public SimulationDocument? Simulation { get; set; }
}

/// <summary>A rigid link with its initial pose and velocity.</summary>
public sealed class LinkDocument
{
    public string Id { get; set; } = string.Empty;

    /// <summary>
    /// <see cref="LinkRoles.Strut"/> for compression members, whose axial
    /// force is checked for tension (PLAN.md 4.1); <see cref="LinkRoles.Link"/>
    /// otherwise.
    /// </summary>
    public string Role { get; set; } = LinkRoles.Link;

    public double Mass { get; set; }

    /// <summary>Moment of inertia about the center of mass, kg·m².</summary>
    public double Inertia { get; set; }

    public double Length { get; set; }

    public PoseDocument Pose { get; set; } = new();

    public TwistDocument Twist { get; set; } = new();
}

public static class LinkRoles
{
    public const string Link = "link";
    public const string Strut = "strut";
}

/// <summary>Center-of-mass position and orientation.</summary>
public sealed class PoseDocument
{
    public double X { get; set; }

    public double Y { get; set; }

    public double Theta { get; set; }
}

/// <summary>Center-of-mass velocity and angular rate.</summary>
public sealed class TwistDocument
{
    public double Vx { get; set; }

    public double Vy { get; set; }

    public double Omega { get; set; }
}

/// <summary>
/// A point on a link (<see cref="Link"/> and <see cref="Point"/> in the link
/// frame) or a fixed ground point (<see cref="Ground"/> in world
/// coordinates). Exactly one form is set.
/// </summary>
public sealed class AttachmentDocument
{
    public string? Link { get; set; }

    /// <summary>Body-frame point [x, y] on <see cref="Link"/>.</summary>
    public double[]? Point { get; set; }

    /// <summary>World point [x, y] fixed to the ground.</summary>
    public double[]? Ground { get; set; }

    [JsonIgnore]
    public bool IsGround => Ground is not null;

    public static AttachmentDocument OnLink(string link, double x, double y) => new() { Link = link, Point = [x, y] };

    public static AttachmentDocument AtGround(double x, double y) => new() { Ground = [x, y] };
}

public sealed class PinDocument
{
    public string Id { get; set; } = string.Empty;

    /// <summary>End A; the reported reaction is the force on this end.</summary>
    public AttachmentDocument A { get; set; } = new();

    public AttachmentDocument B { get; set; } = new();

    /// <summary>Viscous joint friction c, N·m·s/rad; omitted from the file when zero.</summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
    public double Friction { get; set; }

    /// <summary>Stops on the joint angle θ_A − θ_B; null (omitted from the file) for an unlimited joint.</summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public JointLimitDocument? Limit { get; set; }
}

/// <summary>
/// The range of a pin's joint angle φ = θ_A − θ_B (a ground end counts as
/// θ = 0). Past a stop, a torsional spring-damper pushes φ back into range
/// and never pulls it out. Either stop may be omitted, but not both.
/// </summary>
public sealed class JointLimitDocument
{
    /// <summary>Lower stop, rad; null for none.</summary>
    public double? Lower { get; set; }

    /// <summary>Upper stop, rad; null for none.</summary>
    public double? Upper { get; set; }

    /// <summary>Torsional stiffness k, N·m/rad.</summary>
    public double Stiffness { get; set; }

    /// <summary>Torsional damping c, N·m·s/rad.</summary>
    public double Damping { get; set; }
}

/// <summary>
/// A horizontal ground line at y = <see cref="Height"/>: penalty contact
/// along the normal, and Coulomb friction through a tangential
/// spring-damper in series with a slider, at each listed contact point.
/// </summary>
public sealed class GroundDocument
{
    /// <summary>Height of the line, m; with a <see cref="HeightSchedule"/>, its height at t = 0.</summary>
    public double Height { get; set; }

    /// <summary>
    /// For a moving ground (such as a ground drop): the height over time,
    /// piecewise-linear and held before the first point and after the last.
    /// Null (omitted from the file) for a fixed ground.
    /// </summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public List<HeightPointDocument>? HeightSchedule { get; set; }

    /// <summary>Normal penalty stiffness, N/m.</summary>
    public double NormalStiffness { get; set; }

    /// <summary>Normal damping, N·s/m.</summary>
    public double NormalDamping { get; set; }

    /// <summary>Coulomb friction coefficient μ.</summary>
    public double Friction { get; set; }

    /// <summary>Tangential spring stiffness, N/m.</summary>
    public double TangentialStiffness { get; set; }

    /// <summary>Tangential damping, N·s/m; must be positive.</summary>
    public double TangentialDamping { get; set; }

    public List<ContactPointDocument> Contacts { get; set; } = [];
}

/// <summary>
/// A force given in the world frame at a body-fixed point on a link,
/// piecewise-linear in time through <see cref="Schedule"/> and held constant
/// before the first point and after the last.
/// </summary>
public sealed class LoadDocument
{
    public string Id { get; set; } = string.Empty;

    public string Link { get; set; } = string.Empty;

    /// <summary>Body-frame point [x, y] on <see cref="Link"/>.</summary>
    public double[] Point { get; set; } = [0, 0];

    /// <summary>Breakpoints in strictly increasing time.</summary>
    public List<LoadPointDocument> Schedule { get; set; } = [];
}

public sealed class LoadPointDocument
{
    /// <summary>Time, s.</summary>
    public double Time { get; set; }

    /// <summary>World x component of the force, N.</summary>
    public double Fx { get; set; }

    /// <summary>World y component of the force, N.</summary>
    public double Fy { get; set; }
}

public sealed class HeightPointDocument
{
    /// <summary>Time, s.</summary>
    public double Time { get; set; }

    /// <summary>Height of the line, m.</summary>
    public double Height { get; set; }
}

/// <summary>A body-fixed point that can touch the ground.</summary>
public sealed class ContactPointDocument
{
    public string Id { get; set; } = string.Empty;

    public string Link { get; set; } = string.Empty;

    /// <summary>Body-frame point [x, y] on <see cref="Link"/>.</summary>
    public double[] Point { get; set; } = [0, 0];
}

/// <summary>
/// A torque across a pin: +τ on the pin's end A, −τ on end B. The torque is
/// piecewise-linear in time through <see cref="Schedule"/> and held constant
/// before the first point and after the last; one point is a constant torque.
/// </summary>
public sealed class ActuatorDocument
{
    public string Id { get; set; } = string.Empty;

    /// <summary>Id of the pin the torque acts across.</summary>
    public string Pin { get; set; } = string.Empty;

    /// <summary>Breakpoints in strictly increasing time.</summary>
    public List<TorquePointDocument> Schedule { get; set; } = [];
}

public sealed class TorquePointDocument
{
    /// <summary>Time, s.</summary>
    public double Time { get; set; }

    /// <summary>Torque, N·m.</summary>
    public double Torque { get; set; }
}

public sealed class CableDocument
{
    public string Id { get; set; } = string.Empty;

    public AttachmentDocument A { get; set; } = new();

    public AttachmentDocument B { get; set; } = new();

    /// <summary>Rest length L0, m.</summary>
    public double RestLength { get; set; }

    /// <summary>Axial stiffness EA, N.</summary>
    public double AxialStiffness { get; set; }

    /// <summary>Viscous damping c, N·s/m.</summary>
    public double Damping { get; set; }

    /// <summary>
    /// The contractile part of a series-elastic muscle in series with this
    /// cable, or null for a plain cable (PLAN.md M9). With a muscle, the
    /// cable is the muscle's tendon and its rest length is the tendon's own.
    /// </summary>
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public MuscleDocument? Muscle { get; set; }
}

/// <summary>
/// A muscle's contractile part: it shortens at <see cref="ShorteningSpeed"/>
/// times its activation, which is piecewise-linear in time through
/// <see cref="Activation"/>, held before the first point, and must end at zero.
/// </summary>
public sealed class MuscleDocument
{
    /// <summary>Contractile length at t = 0, m.</summary>
    public double InitialLength { get; set; }

    /// <summary>Shortening speed at full activation, m/s.</summary>
    public double ShorteningSpeed { get; set; }

    public List<ActivationPointDocument> Activation { get; set; } = [];
}

public sealed class ActivationPointDocument
{
    /// <summary>s.</summary>
    public double Time { get; set; }

    /// <summary>Activation, from 0 (relaxed) to 1 (full speed).</summary>
    public double Activation { get; set; }
}

/// <summary>Run settings carried by a topology file.</summary>
public sealed class SimulationDocument
{
    /// <summary>Fixed step, s.</summary>
    public double Dt { get; set; }

    /// <summary>Run length, s. Must be a whole number of steps.</summary>
    public double Duration { get; set; }

    /// <summary>Steps between recorded frames.</summary>
    public int FrameStride { get; set; } = 1;

    /// <summary>Bracket width for cable and schedule switch times, s.</summary>
    public double EventTolerance { get; set; } = 1e-12;
}
