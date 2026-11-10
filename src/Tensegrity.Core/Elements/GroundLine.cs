using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Elements;

/// <summary>
/// Tangential state of a ground contact. The mode is a function of the
/// state alone (see <see cref="GroundLine.FrictionModeAt"/>), so the stepper
/// finds every switch as it does for cables (PLAN.md 4.2).
/// </summary>
public enum FrictionMode
{
    /// <summary>The point is off the ground (normal force zero): no friction, and the tangential spring relaxes.</summary>
    Free,

    /// <summary>The slider holds: |k_t·z + c_t·v| ≤ μ·N.</summary>
    Stick,

    /// <summary>The slider moves in +x: the point feels −μ·N along x.</summary>
    SlipPositive,

    /// <summary>The slider moves in −x: the point feels +μ·N along x.</summary>
    SlipNegative,
}

/// <summary>A body-fixed point that can touch the ground line.</summary>
public sealed record ContactPoint
{
    public ContactPoint(string id, string linkId, Vector2d point)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentException.ThrowIfNullOrWhiteSpace(linkId);
        Id = id;
        Attachment = Attachment.OnLink(linkId, point);
    }

    public string Id { get; }

    /// <summary>The point, in its link's body frame.</summary>
    public Attachment Attachment { get; }
}

/// <summary>
/// A horizontal ground line at y = h with penalty contact and Coulomb
/// friction (PLAN.md 4.1), acting on listed body-fixed
/// <see cref="Points"/>. The height h is fixed, or follows a
/// <see cref="HeightSchedule"/> for a moving support such as a ground drop
/// (PLAN.md M6). A moving ground does work N·ḣ on each point it pushes.
/// 
///
/// <b>Normal.</b> With penetration δ = h − y and δ̇ = −ẏ, the ground pushes
/// up with N from <see cref="Normal"/>, a <see cref="ContactLaw"/>: never
/// pulling, with clear, pressing and recoiling modes.
///
/// <b>Tangential.</b> A spring k_t and damper c_t in parallel, in series
/// with a Coulomb slider of strength μ·N. Each point carries the spring's
/// extension z as a state variable (the point's x minus the slider's), and
/// with v the point's x-velocity the slider either holds or slides:
/// <code>
/// stick   |k_t·z + c_t·v| ≤ μ·N    F = −(k_t·z + c_t·v)    ż = v
/// slip σ  otherwise, σ = sign(·)   F = −σ·μ·N              ż = (σ·μ·N − k_t·z)/c_t
/// free    off the ground (N = 0)   F = 0                   ż = −k_t·z/c_t
/// </code>
/// F is the friction force on the point along x. In slip the slider moves
/// at u = v − ż with σ·u &gt; 0, and the spring-damper carries exactly μ·N.
/// The force is continuous at stick–slip switches, z never jumps, and the
/// mode depends only on the state, so stick and slip are ordinary events.
/// A loaded point at rest sticks at a fixed z with no creep.
///
/// <b>Energy.</b> The springs store ½·k·δ² and ½·k_t·z² (elastic). The
/// normal damper, the tangential damper (c_t·ż²) and the slider (μ·N·|u|)
/// dissipate, so the balance closes exactly.
/// </summary>
public sealed class GroundLine
{
    public GroundLine(
        IEnumerable<ContactPoint> points,
        double normalStiffness,
        double normalDamping,
        double friction,
        double tangentialStiffness,
        double tangentialDamping,
        double height = 0,
        Schedule? heightSchedule = null)
    {
        ArgumentNullException.ThrowIfNull(points);
        Points = points.ToArray();
        if (Points.Count == 0)
        {
            throw new ArgumentException("A ground line needs at least one contact point.", nameof(points));
        }

        var duplicate = Points.GroupBy(p => p.Id, StringComparer.Ordinal).FirstOrDefault(g => g.Count() > 1);
        if (duplicate is not null)
        {
            throw new ArgumentException($"Duplicate contact point id '{duplicate.Key}'.", nameof(points));
        }

        if (!double.IsFinite(height))
        {
            throw new ArgumentOutOfRangeException(nameof(height), height, "Must be finite.");
        }

        if (heightSchedule is not null && height != 0)
        {
            throw new ArgumentException("Give a fixed height or a height schedule, not both.", nameof(heightSchedule));
        }

        if (!double.IsFinite(friction) || friction < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(friction), friction, "Must be finite and non-negative.");
        }

        if (!double.IsFinite(tangentialStiffness) || tangentialStiffness <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(tangentialStiffness), tangentialStiffness, "Must be finite and positive.");
        }

        // The slip and free laws divide by c_t: the series damper sets how fast the spring follows the slider.
        if (!double.IsFinite(tangentialDamping) || tangentialDamping <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(tangentialDamping), tangentialDamping, "Must be finite and positive.");
        }

        Normal = new ContactLaw(normalStiffness, normalDamping);
        HeightSchedule = heightSchedule ?? Schedule.Constant(height);
        Friction = friction;
        TangentialStiffness = tangentialStiffness;
        TangentialDamping = tangentialDamping;
    }

    public IReadOnlyList<ContactPoint> Points { get; }

    /// <summary>Height h of the line over time, m; constant for a fixed ground.</summary>
    public Schedule HeightSchedule { get; }

    /// <summary>True if the line moves.</summary>
    public bool IsMoving => !HeightSchedule.IsConstant;

    /// <summary>Height of the line at t = 0, m.</summary>
    public double Height => HeightSchedule.ValueAt(0);

    /// <summary>Normal penalty law: k in N/m, c in N·s/m.</summary>
    public ContactLaw Normal { get; }

    /// <summary>Coulomb friction coefficient μ (static and kinetic alike).</summary>
    public double Friction { get; }

    /// <summary>Tangential spring k_t, N/m.</summary>
    public double TangentialStiffness { get; }

    /// <summary>Tangential damper c_t, N·s/m.</summary>
    public double TangentialDamping { get; }

    /// <summary>Penetration δ = h − y of a point at height y below a line at height h, m.</summary>
    public static double Penetration(double lineHeight, double y) => lineHeight - y;

    /// <summary>
    /// The tangential mode for a point with normal mode
    /// <paramref name="normalMode"/>, normal force N, extension z and
    /// x-velocity v.
    /// </summary>
    public FrictionMode FrictionModeAt(ContactMode normalMode, double normalForce, double extension, double velocity)
    {
        if (normalMode != ContactMode.Pressing)
        {
            return FrictionMode.Free;
        }

        var held = (TangentialStiffness * extension) + (TangentialDamping * velocity);
        var limit = Friction * normalForce;
        if (Math.Abs(held) <= limit)
        {
            return FrictionMode.Stick;
        }

        return held > 0 ? FrictionMode.SlipPositive : FrictionMode.SlipNegative;
    }

    /// <summary>Friction force on the point along x in a fixed mode, N.</summary>
    public double FrictionForce(FrictionMode mode, double normalForce, double extension, double velocity) => mode switch
    {
        FrictionMode.Stick => -((TangentialStiffness * extension) + (TangentialDamping * velocity)),
        FrictionMode.SlipPositive => -Friction * normalForce,
        FrictionMode.SlipNegative => Friction * normalForce,
        FrictionMode.Free => 0.0,
        _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null),
    };

    /// <summary>Rate ż of the spring's extension in a fixed mode, m/s.</summary>
    public double ExtensionRate(FrictionMode mode, double normalForce, double extension, double velocity) => mode switch
    {
        FrictionMode.Stick => velocity,
        FrictionMode.SlipPositive => ((Friction * normalForce) - (TangentialStiffness * extension)) / TangentialDamping,
        FrictionMode.SlipNegative => ((-Friction * normalForce) - (TangentialStiffness * extension)) / TangentialDamping,
        FrictionMode.Free => -TangentialStiffness * extension / TangentialDamping,
        _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null),
    };

    /// <summary>
    /// Power lost in the tangential damper and the slider in a fixed mode,
    /// W: c_t·ż² plus, in slip, μ·N·|u| with u = v − ż the slider's speed.
    /// </summary>
    public double TangentialDissipatedPower(FrictionMode mode, double normalForce, double extension, double velocity)
    {
        var rate = ExtensionRate(mode, normalForce, extension, velocity);
        var power = TangentialDamping * rate * rate;
        var slider = velocity - rate;
        return mode switch
        {
            FrictionMode.SlipPositive => power + (Friction * normalForce * slider),
            FrictionMode.SlipNegative => power - (Friction * normalForce * slider),
            _ => power,
        };
    }

    /// <summary>Energy stored in the tangential spring, ½·k_t·z², J.</summary>
    public double TangentialElasticEnergy(double extension) => 0.5 * TangentialStiffness * extension * extension;
}
