using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A node whose drawing is sized in screen pixels, not run metres, so zooming
/// the camera leaves it the same size. Its position still follows the run.
/// Subclasses draw in run metres after converting pixels with <see cref="Px"/>.
/// </summary>
public abstract partial class ScreenScaled : Node2D
{
    private float pixelsPerUnit = 1f;

    public override void _Ready() => SetProcess(true);

    public override void _Process(double delta)
    {
        // The camera zoom reaches this node through its canvas transform. When it
        // changes, redraw at the new scale so the drawing keeps its pixel size.
        var scale = GetGlobalTransformWithCanvas().X.Length();
        if (scale > 0 && Math.Abs(scale - pixelsPerUnit) > 1e-9f)
        {
            pixelsPerUnit = scale;
            QueueRedraw();
        }
    }

    /// <summary>Converts a length in screen pixels to this node's local units (run metres).</summary>
    protected float Px(float pixels) => pixels / pixelsPerUnit;

    /// <summary>Formats a number with a dot decimal point whatever the system locale.</summary>
    protected static string Num(double value, string format) => value.ToString(format, System.Globalization.CultureInfo.InvariantCulture);

    /// <summary>
    /// Draws text upright on screen, its baseline at <paramref name="offset"/>
    /// (run metres, in the parent's frame, y up) from this node's origin, one
    /// font pixel per screen pixel. The scene flips y and this node may be
    /// rotated, so the text transform undoes both. Leaves the transform reset.
    /// </summary>
    protected void DrawUpright(Vector2 offset, string text, Color color, int fontPixels)
    {
        var k = Px(fontPixels) / fontPixels;
        var anchor = offset.Rotated(-Rotation);
        DrawSetTransform(anchor, -Rotation, new Vector2(k, -k));
        DrawString(ThemeDB.FallbackFont, Vector2.Zero, text, HorizontalAlignment.Left, -1, fontPixels, color);
        DrawSetTransform(Vector2.Zero, 0, Vector2.One);
    }
}
