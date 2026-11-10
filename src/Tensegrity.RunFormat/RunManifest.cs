namespace Tensegrity.RunFormat;

/// <summary>
/// <c>manifest.json</c> of a run directory (PLAN.md 6): what was simulated,
/// how, the layout of <c>frames.bin</c>, and diagnostics. Diagnostics are
/// float64; frames are float32.
/// </summary>
public sealed class RunManifest
{
    public const int CurrentFormatVersion = 1;

    public int FormatVersion { get; set; } = CurrentFormatVersion;

    /// <summary>ISO 8601 UTC time the run was written.</summary>
    public string CreatedUtc { get; set; } = string.Empty;

    /// <summary>Git commit of the simulator, or "unknown".</summary>
    public string SimVersion { get; set; } = "unknown";

    /// <summary>True if the working tree had uncommitted changes; null if unknown.</summary>
    public bool? SimDirty { get; set; }

    public int Dimensions { get; set; } = 2;

    public Dictionary<string, string> Units { get; set; } = new()
    {
        ["length"] = "m",
        ["mass"] = "kg",
        ["time"] = "s",
        ["angle"] = "rad",
        ["force"] = "N",
        ["energy"] = "J",
    };

    public string Integrator { get; set; } = string.Empty;

    public string ConstraintMethod { get; set; } = string.Empty;

    /// <summary>Baumgarte natural frequency, rad/s.</summary>
    public double ConstraintStabilization { get; set; }

    public double Dt { get; set; }

    public double Duration { get; set; }

    public long TotalSteps { get; set; }

    public int FrameStride { get; set; }

    public int FrameCount { get; set; }

    /// <summary>Bracket width for cable and schedule switch times, s.</summary>
    public double EventTolerance { get; set; }

    public double[] Gravity { get; set; } = [0, 0];

    /// <summary>
    /// Ground contact model, <see cref="GroundModels.PenaltyCoulomb"/>, with its
    /// parameters in <c>topology.ground</c>; null when the run has no ground.
    /// </summary>
    public string? Ground { get; set; }

    /// <summary>Global settling damping, 1/s; 0 in realistic runs (PLAN.md 4.3).</summary>
    public double GlobalDamping { get; set; }

    /// <summary>
    /// Joint actuators and their schedules, the same list as
    /// <c>topology.actuators</c>; empty when the run has none.
    /// </summary>
    public List<ActuatorDocument> Actuators { get; set; } = [];

    public long Seed { get; set; }

    /// <summary>The topology file the run was loaded from, if any.</summary>
    public string? TopologySource { get; set; }

    /// <summary>The simulated topology, with its initial state.</summary>
    public TopologyDocument Topology { get; set; } = new();

    public FrameLayout FrameLayout { get; set; } = new();

    public RunDiagnostics Diagnostics { get; set; } = new();
}

/// <summary>
/// Layout of <c>frames.bin</c>: <see cref="FrameCount"/> frames one after
/// another, each <see cref="Channels"/>.Count little-endian float32 values in
/// channel order. Channel names are listed in <see cref="FrameChannels"/>.
/// </summary>
public sealed class FrameLayout
{
    public string ByteOrder { get; set; } = "little";

    public string Scalar { get; set; } = "float32";

    public List<string> Channels { get; set; } = [];

    public int FrameBytes => 4 * Channels.Count;
}

public static class GroundModels
{
    /// <summary>Penalty spring-damper normal; Coulomb friction via a tangential spring-damper in series with a slider.</summary>
    public const string PenaltyCoulomb = "penalty_spring_damper_coulomb_slider";
}

/// <summary>Names of the per-frame channels.</summary>
public static class FrameChannels
{
    public const string Time = "time";

    public static string LinkX(string link) => $"link.{link}.x";

    public static string LinkY(string link) => $"link.{link}.y";

    public static string LinkTheta(string link) => $"link.{link}.theta";

    public static string CableTension(string cable) => $"cable.{cable}.tension";

    /// <summary>World endpoints of a cable, so renderers place recorded numbers and compute nothing.</summary>
    public static string CableAx(string cable) => $"cable.{cable}.ax";

    public static string CableAy(string cable) => $"cable.{cable}.ay";

    public static string CableBx(string cable) => $"cable.{cable}.bx";

    public static string CableBy(string cable) => $"cable.{cable}.by";

    /// <summary>Force the pin exerts on its end A, N.</summary>
    public static string PinFx(string pin) => $"pin.{pin}.fx";

    public static string PinFy(string pin) => $"pin.{pin}.fy";

    /// <summary>World position of a pin (its end A), so renderers can draw it without geometry.</summary>
    public static string PinX(string pin) => $"pin.{pin}.x";

    public static string PinY(string pin) => $"pin.{pin}.y";

    /// <summary>Torque a pin's joint limit applies to its end A, N·m (end B takes the opposite). Only pins with a limit have it.</summary>
    public static string PinLimitTorque(string pin) => $"pin.{pin}.limit_torque";

    /// <summary>World force of an external load at that frame's time, N.</summary>
    public static string LoadFx(string load) => $"load.{load}.fx";

    public static string LoadFy(string load) => $"load.{load}.fy";

    /// <summary>World position of the point a load acts at.</summary>
    public static string LoadX(string load) => $"load.{load}.x";

    public static string LoadY(string load) => $"load.{load}.y";

    /// <summary>Height of the ground line at that frame, m; present whenever the run has a ground.</summary>
    public const string GroundHeight = "ground.height";

    /// <summary>Normal force the ground exerts on a contact point, N (upward, never negative).</summary>
    public static string ContactNormal(string contact) => $"contact.{contact}.normal";

    /// <summary>Friction force the ground exerts on a contact point along +x, N.</summary>
    public static string ContactFriction(string contact) => $"contact.{contact}.friction";

    /// <summary>World position of a contact point, so renderers can draw it without geometry.</summary>
    public static string ContactX(string contact) => $"contact.{contact}.x";

    public static string ContactY(string contact) => $"contact.{contact}.y";

    /// <summary>Torque an actuator applies at that frame's time, N·m: +τ on its pin's end A.</summary>
    public static string ActuatorTorque(string actuator) => $"actuator.{actuator}.torque";
}

public sealed class RunDiagnostics
{
    /// <summary>Energy at every recorded frame, float64.</summary>
    public EnergySeries Energy { get; set; } = new();

    /// <summary>Worst |E_total(t) − E_total(0)| over every step, J.</summary>
    public double MaxEnergyBalanceError { get; set; }

    /// <summary>Worst pin position error over every step, m.</summary>
    public double MaxPinPositionResidual { get; set; }

    /// <summary>Worst pin velocity error over every step, m/s.</summary>
    public double MaxPinVelocityResidual { get; set; }

    /// <summary>Steps split at a cable, joint-limit or ground contact mode switch, or an actuator or load schedule breakpoint.</summary>
    public long EventCount { get; set; }

    /// <summary>Steps that hit the per-step switch cap and finished unsplit.</summary>
    public long UnresolvedEventSteps { get; set; }

    /// <summary>A strut whose minimum compression falls below −this is flagged as in tension, N.</summary>
    public double CompressionTolerance { get; set; }

    /// <summary>Midpoint axial force range per link over every step.</summary>
    public List<LinkAxialForce> AxialForces { get; set; } = [];

    /// <summary>Ids of struts that went into tension: a configuration error for the topology (PLAN.md 4.1).</summary>
    public List<string> StrutsInTension { get; set; } = [];

    /// <summary>Tension range per cable over every step.</summary>
    public List<CableTensionRange> CableTensions { get; set; } = [];
}

public sealed class EnergySeries
{
    public List<double> Time { get; set; } = [];

    public List<double> Kinetic { get; set; } = [];

    public List<double> Gravitational { get; set; } = [];

    /// <summary>Energy stored in stretched cables, compressed joint-limit stops and ground contact springs.</summary>
    public List<double> Elastic { get; set; } = [];

    /// <summary>Energy lost to cable damping, joint-limit damping, joint friction and ground contact so far.</summary>
    public List<double> Dissipated { get; set; } = [];

    /// <summary>Work done by the joint actuators so far.</summary>
    public List<double> ActuatorWork { get; set; } = [];

    /// <summary>Work done by external loads so far.</summary>
    public List<double> ExternalWork { get; set; } = [];

    /// <summary>Kinetic + gravitational + elastic + dissipated − actuator work − external work: constant up to integration error.</summary>
    public List<double> Total { get; set; } = [];
}

/// <summary>
/// Axial force across a link's midpoint section, compression positive, N:
/// the force the link's −x half exerts on its +x half along the link axis.
/// </summary>
public sealed class LinkAxialForce
{
    public string Link { get; set; } = string.Empty;

    public string Role { get; set; } = LinkRoles.Link;

    public double MinCompression { get; set; }

    public double MaxCompression { get; set; }
}

public sealed class CableTensionRange
{
    public string Cable { get; set; } = string.Empty;

    public double MinTension { get; set; }

    public double MaxTension { get; set; }
}
