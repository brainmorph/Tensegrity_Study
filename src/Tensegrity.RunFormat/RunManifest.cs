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

    /// <summary>Bracket width for cable switch times, s.</summary>
    public double EventTolerance { get; set; }

    public double[] Gravity { get; set; } = [0, 0];

    /// <summary>Ground contact model; null while there is none (deferred to M6).</summary>
    public string? Ground { get; set; }

    /// <summary>Global settling damping, 1/s; 0 in realistic runs (PLAN.md 4.3).</summary>
    public double GlobalDamping { get; set; }

    /// <summary>Joint actuators and their schedules; none before M5.</summary>
    public List<string> Actuators { get; set; } = [];

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

    /// <summary>Steps split at a cable mode switch.</summary>
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

    public List<double> Elastic { get; set; } = [];

    public List<double> Dissipated { get; set; } = [];

    /// <summary>Kinetic + gravitational + elastic + dissipated: constant up to integration error.</summary>
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
