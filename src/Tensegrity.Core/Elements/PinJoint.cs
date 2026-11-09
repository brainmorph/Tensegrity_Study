namespace Tensegrity.Core.Elements;

/// <summary>
/// A pin joint: the holonomic constraint that two attachment points
/// coincide. Either end may be the ground (a fixed pivot), but not both.
/// The constraint force is solved for with a Lagrange multiplier, so it is
/// part of the solution rather than an input (PLAN.md 4.1, 9.1).
///
/// The reaction reported for a pin is the force it exerts on end
/// <see cref="A"/>; end <see cref="B"/> receives the opposite force.
/// </summary>
public sealed class PinJoint
{
    public PinJoint(string id, Attachment a, Attachment b)
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

        Id = id;
        A = a;
        B = b;
    }

    public string Id { get; }

    public Attachment A { get; }

    public Attachment B { get; }
}
