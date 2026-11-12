using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// The fixed body that ground-anchored cables pull on, drawn so an anchor
/// does not float in space. Each cable anchor on the ground is joined to
/// the nearest pin on the ground by a bar: both are fixed to the same
/// body, as the shoulder tendon's anchor and the shoulder are. An anchor
/// with no ground pin gets a fixed base under it instead. The points come
/// from the topology and never move; the node's transform stays identity.
/// </summary>
public partial class BodyView : Node2D
{
    private static readonly Color BodyColor = new("8a919c");

    /// <summary>Cable anchors on the ground, run space, m.</summary>
    public IReadOnlyList<Vector2> Anchors { get; set; } = [];

    /// <summary>Pins to the ground, run space, m.</summary>
    public IReadOnlyList<Vector2> GroundPins { get; set; } = [];

    /// <summary>Bar width, m.</summary>
    public float Stroke { get; set; } = 0.01f;

    public override void _Draw()
    {
        foreach (var anchor in Anchors)
        {
            if (GroundPins.Count > 0)
            {
                var nearest = GroundPins.MinBy(p => p.DistanceSquaredTo(anchor));
                DrawLine(anchor, nearest, BodyColor, Stroke);
                DrawCircle(anchor, Stroke / 2, BodyColor, filled: true);
                DrawCircle(nearest, Stroke / 2, BodyColor, filled: true);
            }
            else
            {
                // Points are run-space (y-up), so the base sits over the anchor, as a ceiling mount.
                var r = Stroke;
                DrawLine(anchor + new Vector2(-2.6f * r, 0), anchor + new Vector2(2.6f * r, 0), BodyColor, r * 0.4f);
                for (var i = -2; i <= 2; i++)
                {
                    var foot = anchor + new Vector2(i * r, 0);
                    DrawLine(foot, foot + new Vector2(0.8f * r, 0.9f * r), BodyColor, r * 0.25f);
                }
            }
        }
    }
}
