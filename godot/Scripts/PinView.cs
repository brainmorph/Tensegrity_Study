using Godot;

namespace Tensegrity.Viewer;

/// <summary>A pin at its recorded world position; ground pins get a base.</summary>
public partial class PinView : Node2D
{
    private static readonly Color Ring = new("ffffff");
    private static readonly Color Hub = new("16181d");
    private static readonly Color Base = new("8a919c");

    public string Id { get; set; } = string.Empty;

    public bool IsGround { get; set; }

    /// <summary>Dot radius, m.</summary>
    public float Radius { get; set; } = 0.01f;

    public override void _Draw()
    {
        if (IsGround)
        {
            // Points are run-space (y-up), so the base sits under the pin on screen.
            var r = Radius;
            DrawColoredPolygon([new Vector2(0, 0), new Vector2(-1.8f * r, -2.4f * r), new Vector2(1.8f * r, -2.4f * r)], Base);
            DrawLine(new Vector2(-2.6f * r, -2.4f * r), new Vector2(2.6f * r, -2.4f * r), Base, r * 0.4f);
        }

        DrawCircle(Vector2.Zero, Radius, Ring, filled: true);
        DrawCircle(Vector2.Zero, Radius * 0.5f, Hub, filled: true);
    }
}
