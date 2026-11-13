using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A recorded force drawn as an arrow from its point of application. The
/// arrow's length is set by the force's magnitude against the run's largest
/// force, in screen pixels, and its label gives the value in newtons.
/// </summary>
public partial class ForceArrow : ScreenScaled
{
    private const float MaxLengthPx = 90f;
    private const float MinLengthPx = 6f;
    private const float ShaftWidth = 2.5f;
    private const float HeadPx = 10f;

    /// <summary>Largest force magnitude in the run, N; the longest arrow is this force.</summary>
    public double MaxForce { get; set; } = 1;

    /// <summary>The colour of the arrow and its label.</summary>
    public Color Tint { get; set; } = new("7aa2f7");

    /// <summary>The label's name, e.g. "elbow".</summary>
    public string Caption { get; set; } = string.Empty;

    /// <summary>The force on show, N, in run axes (x right, y up).</summary>
    public Vector2 Force { get; private set; }

    /// <summary>Sets the force shown and redraws; the point of application is the node's position.</summary>
    public void Show(Vector2 force)
    {
        Force = force;
        QueueRedraw();
    }

    public override void _Draw()
    {
        var magnitude = Force.Length();
        if (magnitude < 1e-9)
        {
            return;
        }

        var direction = Force / magnitude;
        var fraction = Mathf.Clamp((float)(magnitude / MaxForce), 0, 1);
        var lengthPx = MinLengthPx + (MaxLengthPx - MinLengthPx) * fraction;
        var tip = direction * Px(lengthPx);

        DrawLine(Vector2.Zero, tip, Tint, Px(ShaftWidth), true);

        var head = -direction * Px(HeadPx);
        var side = new Vector2(-head.Y, head.X) * 0.5f;
        DrawColoredPolygon([tip, tip + head + side, tip + head - side], Tint);

        // The label sits just past the tip, in the arrow's direction.
        var anchor = tip + direction * Px(6f) + new Vector2(0, Px(4f));
        DrawUpright(anchor, $"{Caption} {Num(magnitude, "0.0")} N", Tint, 13);
    }
}
