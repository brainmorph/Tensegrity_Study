using System.Globalization;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Analysis;

/// <summary>A tendon from a fixed ground anchor to one hinge of a five-bar.</summary>
/// <param name="Anchor">World point of the ground end, m.</param>
/// <param name="RestLength">Rest length L0, m.</param>
public readonly record struct FiveBarTendon(Vector2d Anchor, double RestLength);

/// <summary>
/// A planar five-bar held by three tendons, the first candidate for a
/// shape with four passively stable states cycled by one servo (see
/// brainstorm-multistable-servo.md, Candidate 1). SI units.
/// <code>
///                    E
///          tendon E ╱ ╲
///         coupler C╱   ╲coupler D
///                 C     D
///   tendon C     ╱       ╲     tendon D
///         servo ╱         ╲ passive
///              A ━━━━━━━━━ B          (ground)
/// </code>
/// A (the servo) is at the origin and B (a plain hinge) at
/// (<see cref="BaseWidth"/>, 0); gravity points along −y. Links A–C, C–E,
/// D–E and B–D are slender bars of <see cref="LinkMass"/> each. Tendons C,
/// D and E run from ground anchors to the hinges they are named after, and
/// tendon C–E to a point on coupler C–E. Each pulls only when stretched,
/// with stiffness k = EA / L0, the simulator's cable law.
/// </summary>
public sealed record FiveBarGeometry
{
    public double BaseWidth { get; init; } = 0.40;

    /// <summary>Link A–C, turned by the servo at A, m.</summary>
    public double ServoLength { get; init; } = 0.20;

    /// <summary>Link C–E, m.</summary>
    public double CouplerCLength { get; init; } = 0.20;

    /// <summary>Link D–E, m.</summary>
    public double CouplerDLength { get; init; } = 0.20;

    /// <summary>Link B–D, free to turn at B, m.</summary>
    public double PassiveLength { get; init; } = 0.20;

    /// <summary>Mass of each link, kg.</summary>
    public double LinkMass { get; init; } = 0.02;

    /// <summary>Axial stiffness EA of every tendon, N.</summary>
    public double AxialStiffness { get; init; } = 500;

    /// <summary>Viscous damping of every tendon, N·s/m. Used by simulations only; statics ignore it.</summary>
    public double TendonDamping { get; init; } = 1.0;

    /// <summary>Gravitational acceleration along −y, m/s².</summary>
    public double Gravity { get; init; } = 9.81;

    // The defaults are a starting point, not a design: the tendons are a few
    // millimetres short of reaching a symmetric arch (C at 60°, D at 120°),
    // which gives three minima, two of them mirror images.

    /// <summary>Tendon from a ground anchor to hinge C.</summary>
    public FiveBarTendon TendonC { get; init; } = new(new Vector2d(-0.10, 0.30), 0.23);

    /// <summary>Tendon from a ground anchor to hinge D.</summary>
    public FiveBarTendon TendonD { get; init; } = new(new Vector2d(0.50, 0.30), 0.23);

    /// <summary>Tendon from a ground anchor to hinge E.</summary>
    public FiveBarTendon TendonE { get; init; } = new(new Vector2d(0.20, 0.55), 0.24);

    /// <summary>
    /// The fourth tendon, from a ground anchor to a point on coupler C–E
    /// (<see cref="TendonCEPosition"/>). That point moves with both the
    /// servo and the coupler, so this tendon cannot fix the servo angle by
    /// itself, as tendon C does. By default it is just slack in the
    /// symmetric arch, so the three-tendon minima stand.
    /// </summary>
    public FiveBarTendon TendonCE { get; init; } = new(new Vector2d(-0.05, 0.50), 0.32);

    /// <summary>Where tendon C–E attaches, as a fraction of the coupler from C (0) to E (1).</summary>
    public double TendonCEPosition { get; init; } = 0.5;

    /// <summary>Throws if any length, mass or stiffness is not finite and positive, or gravity is negative.</summary>
    public void Validate()
    {
        Positive(BaseWidth, nameof(BaseWidth));
        Positive(ServoLength, nameof(ServoLength));
        Positive(CouplerCLength, nameof(CouplerCLength));
        Positive(CouplerDLength, nameof(CouplerDLength));
        Positive(PassiveLength, nameof(PassiveLength));
        Positive(LinkMass, nameof(LinkMass));
        Positive(AxialStiffness, nameof(AxialStiffness));
        if (!double.IsFinite(TendonDamping) || TendonDamping < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(TendonDamping), TendonDamping, "Must be finite and not negative.");
        }

        if (!double.IsFinite(Gravity) || Gravity < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(Gravity), Gravity, "Must be finite and not negative.");
        }

        if (!(TendonCEPosition >= 0 && TendonCEPosition <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(TendonCEPosition), TendonCEPosition, "Must lie on the coupler, in [0, 1].");
        }

        foreach (var (name, tendon) in new[] { (nameof(TendonC), TendonC), (nameof(TendonD), TendonD), (nameof(TendonE), TendonE), (nameof(TendonCE), TendonCE) })
        {
            Positive(tendon.RestLength, name + ".RestLength");
            if (!double.IsFinite(tendon.Anchor.X) || !double.IsFinite(tendon.Anchor.Y))
            {
                throw new ArgumentOutOfRangeException(name, tendon.Anchor, "The anchor must be finite.");
            }
        }

        static void Positive(double value, string name)
        {
            if (!double.IsFinite(value) || value <= 0)
            {
                throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
            }
        }
    }
}

/// <summary>
/// One number of a <see cref="FiveBarGeometry"/>, by the name the
/// command line uses: <c>base_width</c>, <c>servo_length</c>,
/// <c>coupler_c_length</c>, <c>coupler_d_length</c>, <c>passive_length</c>,
/// <c>link_mass</c>, <c>axial_stiffness</c>, <c>gravity</c>, and for each
/// tendon <c>tendon_c.x</c>, <c>tendon_c.y</c> (its anchor, m) and
/// <c>tendon_c.rest</c> (its rest length, m), likewise <c>tendon_d</c> and
/// <c>tendon_e</c> and <c>tendon_ce</c>, which also has <c>tendon_ce.at</c>
/// (where it attaches, as a fraction of coupler C–E from C). Values are
/// absolute, not offsets.
/// </summary>
public sealed record FiveBarParameter(string Name)
{
    public static IReadOnlyList<string> Names { get; } =
    [
        "base_width", "servo_length", "coupler_c_length", "coupler_d_length", "passive_length", "link_mass", "axial_stiffness", "gravity",
        "tendon_c.x", "tendon_c.y", "tendon_c.rest", "tendon_d.x", "tendon_d.y", "tendon_d.rest", "tendon_e.x", "tendon_e.y", "tendon_e.rest",
        "tendon_ce.x", "tendon_ce.y", "tendon_ce.rest", "tendon_ce.at",
    ];

    public static FiveBarParameter Parse(string name)
    {
        ArgumentNullException.ThrowIfNull(name);
        return Names.Contains(name, StringComparer.Ordinal)
            ? new FiveBarParameter(name)
            : throw new FormatException($"Unknown five-bar parameter '{name}'. Known: {string.Join(", ", Names)}.");
    }

    /// <summary>
    /// Parses <c>NAME=VALUE</c> pairs separated by commas, as in
    /// <c>tendon_c.x=-0.1,tendon_c.rest=0.12</c>, and applies them in order.
    /// </summary>
    public static FiveBarGeometry ApplyAll(FiveBarGeometry geometry, string assignments)
    {
        ArgumentNullException.ThrowIfNull(assignments);
        foreach (var pair in assignments.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            var equals = pair.IndexOf('=', StringComparison.Ordinal);
            if (equals < 0 || !double.TryParse(pair[(equals + 1)..], NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
            {
                throw new FormatException($"'{pair}' is not NAME=NUMBER.");
            }

            geometry = Parse(pair[..equals]).Apply(geometry, value);
        }

        return geometry;
    }

    /// <summary>The geometry with this parameter set to <paramref name="value"/>.</summary>
    public FiveBarGeometry Apply(FiveBarGeometry geometry, double value)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        return Name switch
        {
            "base_width" => geometry with { BaseWidth = value },
            "servo_length" => geometry with { ServoLength = value },
            "coupler_c_length" => geometry with { CouplerCLength = value },
            "coupler_d_length" => geometry with { CouplerDLength = value },
            "passive_length" => geometry with { PassiveLength = value },
            "link_mass" => geometry with { LinkMass = value },
            "axial_stiffness" => geometry with { AxialStiffness = value },
            "gravity" => geometry with { Gravity = value },
            "tendon_c.x" => geometry with { TendonC = geometry.TendonC with { Anchor = geometry.TendonC.Anchor with { X = value } } },
            "tendon_c.y" => geometry with { TendonC = geometry.TendonC with { Anchor = geometry.TendonC.Anchor with { Y = value } } },
            "tendon_c.rest" => geometry with { TendonC = geometry.TendonC with { RestLength = value } },
            "tendon_d.x" => geometry with { TendonD = geometry.TendonD with { Anchor = geometry.TendonD.Anchor with { X = value } } },
            "tendon_d.y" => geometry with { TendonD = geometry.TendonD with { Anchor = geometry.TendonD.Anchor with { Y = value } } },
            "tendon_d.rest" => geometry with { TendonD = geometry.TendonD with { RestLength = value } },
            "tendon_e.x" => geometry with { TendonE = geometry.TendonE with { Anchor = geometry.TendonE.Anchor with { X = value } } },
            "tendon_e.y" => geometry with { TendonE = geometry.TendonE with { Anchor = geometry.TendonE.Anchor with { Y = value } } },
            "tendon_e.rest" => geometry with { TendonE = geometry.TendonE with { RestLength = value } },
            "tendon_ce.x" => geometry with { TendonCE = geometry.TendonCE with { Anchor = geometry.TendonCE.Anchor with { X = value } } },
            "tendon_ce.y" => geometry with { TendonCE = geometry.TendonCE with { Anchor = geometry.TendonCE.Anchor with { Y = value } } },
            "tendon_ce.rest" => geometry with { TendonCE = geometry.TendonCE with { RestLength = value } },
            "tendon_ce.at" => geometry with { TendonCEPosition = value },
            _ => throw new InvalidOperationException(Name),
        };
    }

    public override string ToString() => Name;
}
