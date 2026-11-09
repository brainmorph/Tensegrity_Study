namespace Tensegrity.Core.Elements;

/// <summary>
/// A pin joint: the holonomic constraint that two attachment points
/// coincide. Either end may be the ground (a fixed pivot), but not both.
/// The constraint force is solved for with a Lagrange multiplier, so it is
/// part of the solution rather than an input (PLAN.md 4.1, 9.1).
///
/// The reaction reported for a pin is the force it exerts on end
/// <see cref="A"/>; end <see cref="B"/> receives the opposite force.
///
/// A pin may have viscous joint friction (PLAN.md 4.3): with relative rate
/// Δω = ω_A − ω_B it applies −c·Δω to end A and +c·Δω to end B, a couple
/// that resists the relative rotation and dissipates c·Δω². It is off (zero)
/// by default. Viscous rather than Coulomb friction keeps the right-hand
/// side smooth, so it needs no events.
/// </summary>
public sealed class PinJoint
{
    public PinJoint(string id, Attachment a, Attachment b, double friction = 0)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        if (a.IsGround && b.IsGround)
        {
            throw new ArgumentException($"Pin '{id}' joins the ground to itself.");
        }

        if (!a.IsGround && a.LinkId == b.LinkId)
        {
            throw new ArgumentException($"Pin '{id}' joins link '{a.LinkId}' to itself.");
        }

        if (!double.IsFinite(friction) || friction < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(friction), friction, "Must be finite and non-negative.");
        }

        Id = id;
        A = a;
        B = b;
        Friction = friction;
    }

    public string Id { get; }

    public Attachment A { get; }

    public Attachment B { get; }

    /// <summary>Viscous joint friction coefficient c, N·m·s/rad; zero for a frictionless pin.</summary>
    public double Friction { get; }
}
