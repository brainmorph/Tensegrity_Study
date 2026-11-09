using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Elements;

/// <summary>
/// Where a pin or cable end is fixed: a body-fixed point on a link (in that
/// link's body frame), or a fixed point of the ground frame (in world
/// coordinates). Body-fixed points let forces act at the true point and
/// produce moments (PLAN.md 4.1).
/// </summary>
public readonly record struct Attachment
{
    private Attachment(string? linkId, Vector2d point)
    {
        LinkId = linkId;
        Point = point;
    }

    /// <summary>The link this point belongs to, or null for the ground frame.</summary>
    public string? LinkId { get; }

    /// <summary>Body-frame point for a link, world point for the ground.</summary>
    public Vector2d Point { get; }

    public bool IsGround => LinkId is null;

    public static Attachment Ground(Vector2d worldPoint)
    {
        CheckFinite(worldPoint);
        return new Attachment(null, worldPoint);
    }

    public static Attachment OnLink(string linkId, Vector2d localPoint)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(linkId);
        CheckFinite(localPoint);
        return new Attachment(linkId, localPoint);
    }

    public override string ToString() =>
        IsGround ? $"ground{Point}" : $"{LinkId}{Point}";

    private static void CheckFinite(Vector2d point)
    {
        if (!double.IsFinite(point.X) || !double.IsFinite(point.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(point), point, "Must be finite.");
        }
    }
}
