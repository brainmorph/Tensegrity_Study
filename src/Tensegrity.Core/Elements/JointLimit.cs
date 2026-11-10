namespace Tensegrity.Core.Elements;

/// <summary>
/// The range of motion of a pin joint: stops on its joint angle
/// φ = θ_A − θ_B, the angle of end A's link relative to end B's (a ground
/// end counts as θ = 0). Angles are not wrapped, so φ is continuous.
///
/// Each stop is a stiff torsional <see cref="ContactLaw"/>. Past the lower
/// stop the penetration is δ = <see cref="Lower"/> − φ, and the stop
/// applies +F to end A and −F to end B; past the upper stop δ = φ −
/// <see cref="Upper"/>, with the signs reversed. The pair is a pure couple,
/// like a joint actuator, so it adds no net force and keeps the total
/// angular momentum of a free system. Either stop may be left open
/// (infinite); at least one must be finite.
///
/// This stands in for bone-on-bone contact at a joint: two links pinned
/// together can only meet by folding about their shared pin, so a stop on
/// the joint angle keeps them from passing through each other.
/// </summary>
public sealed class JointLimit
{
    public JointLimit(double lower, double upper, double stiffness, double damping = 0)
    {
        if (double.IsNaN(lower) || double.IsPositiveInfinity(lower))
        {
            throw new ArgumentOutOfRangeException(nameof(lower), lower, "Must be finite or negative infinity.");
        }

        if (double.IsNaN(upper) || double.IsNegativeInfinity(upper))
        {
            throw new ArgumentOutOfRangeException(nameof(upper), upper, "Must be finite or positive infinity.");
        }

        if (!(lower < upper))
        {
            throw new ArgumentException($"The lower stop {lower} must be below the upper stop {upper}.");
        }

        if (!double.IsFinite(lower) && !double.IsFinite(upper))
        {
            throw new ArgumentException("A joint limit needs at least one finite stop.");
        }

        Lower = lower;
        Upper = upper;
        Stop = new ContactLaw(stiffness, damping);
    }

    /// <summary>Lowest joint angle before the stop engages, rad; −∞ for no lower stop.</summary>
    public double Lower { get; }

    /// <summary>Highest joint angle before the stop engages, rad; +∞ for no upper stop.</summary>
    public double Upper { get; }

    /// <summary>The stops' torsional spring-damper (k in N·m/rad, c in N·m·s/rad).</summary>
    public ContactLaw Stop { get; }

    /// <summary>Torsional stiffness k, N·m/rad.</summary>
    public double Stiffness => Stop.Stiffness;

    /// <summary>Torsional damping c, N·m·s/rad.</summary>
    public double Damping => Stop.Damping;

    public bool HasLower => double.IsFinite(Lower);

    public bool HasUpper => double.IsFinite(Upper);

    /// <summary>Penetration past the lower stop at joint angle φ, rad; −∞ without a lower stop.</summary>
    public double LowerPenetration(double angle) => Lower - angle;

    /// <summary>Penetration past the upper stop at joint angle φ, rad; −∞ without an upper stop.</summary>
    public double UpperPenetration(double angle) => angle - Upper;

    /// <summary>
    /// The physical torque on end A at joint angle φ and rate φ̇, N·m: positive
    /// at the lower stop, negative at the upper, zero in between.
    /// </summary>
    public double Torque(double angle, double rate) =>
        TorqueInModes(LowerMode(angle, rate), UpperMode(angle, rate), angle, rate);

    /// <summary>Mode of the lower stop; always clear without one.</summary>
    public ContactMode LowerMode(double angle, double rate) =>
        HasLower ? Stop.ModeAt(LowerPenetration(angle), -rate) : ContactMode.Clear;

    /// <summary>Mode of the upper stop; always clear without one.</summary>
    public ContactMode UpperMode(double angle, double rate) =>
        HasUpper ? Stop.ModeAt(UpperPenetration(angle), rate) : ContactMode.Clear;

    /// <summary>The torque on end A with both stops held in the given modes, N·m.</summary>
    public double TorqueInModes(ContactMode lower, ContactMode upper, double angle, double rate)
    {
        var torque = 0.0;
        if (lower != ContactMode.Clear)
        {
            torque += Stop.ForceInMode(lower, LowerPenetration(angle), -rate);
        }

        if (upper != ContactMode.Clear)
        {
            torque -= Stop.ForceInMode(upper, UpperPenetration(angle), rate);
        }

        return torque;
    }

    /// <summary>Energy stored in whichever stop is compressed, J.</summary>
    public double ElasticEnergy(double angle) =>
        (HasLower ? Stop.ElasticEnergy(LowerPenetration(angle)) : 0.0)
        + (HasUpper ? Stop.ElasticEnergy(UpperPenetration(angle)) : 0.0);

    /// <summary>Power lost in the stops' dampers with both held in the given modes, W.</summary>
    public double DissipatedPowerInModes(ContactMode lower, ContactMode upper, double angle, double rate)
    {
        var power = 0.0;
        if (lower != ContactMode.Clear)
        {
            power += Stop.DissipatedPowerInMode(lower, LowerPenetration(angle), -rate);
        }

        if (upper != ContactMode.Clear)
        {
            power += Stop.DissipatedPowerInMode(upper, UpperPenetration(angle), rate);
        }

        return power;
    }
}
