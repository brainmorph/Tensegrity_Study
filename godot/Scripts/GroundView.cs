using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// The ground line, with hatching on the solid side. The node sits at the
/// line's recorded height for the frame on show (it moves in a ground drop)
/// and draws the line at its own origin, so it places one recorded number
/// and computes nothing.
/// </summary>
public partial class GroundView : Node2D
{
    private static readonly Color LineColor = new("8a919c");
    private static readonly Color HatchColor = new("4a505a");

    /// <summary>Left and right ends of the drawn line, m.</summary>
    public float Left { get; set; } = -1;

    public float Right { get; set; } = 1;

    /// <summary>Line width, m.</summary>
    public float Stroke { get; set; } = 0.01f;

    public override void _Draw()
    {
        // Points are run-space (y-up), so the hatching sits below the line on screen.
        var spacing = Stroke * 4;
        var depth = Stroke * 3;
        for (var x = Left; x < Right; x += spacing)
        {
            DrawLine(new Vector2(x, 0), new Vector2(x - depth, -depth), HatchColor, Stroke * 0.4f);
        }

        DrawLine(new Vector2(Left, 0), new Vector2(Right, 0), LineColor, Stroke * 0.6f);
    }
}
