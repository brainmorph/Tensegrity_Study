using System.Globalization;
using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A servo at its pin: a housing ring around the joint, an arc showing the
/// torque it applies, and a readout of that torque beside it. The torque is
/// the recorded actuator value of the frame on show; +τ acts on the pin's end A
/// (counter-clockwise in run space), −τ on end B.
///
/// Its size is in screen pixels, not run metres, so zooming the camera into
/// the structure leaves the servo graphics the same size. The pin it sits on
/// still moves with the structure.
/// </summary>
public partial class ActuatorView : Node2D
{
    private static readonly Color Housing = new("f2b544");
    private static readonly Color Arc = new("ffd27a");
    private static readonly Color Faint = new("f2b544", 0.35f);

    // Sizes in screen pixels.
    private const float HousingRadius = 14f;
    private const float HousingWidth = 3.5f;
    private const float ArcRadius = 26f;
    private const float ArcWidth = 3.5f;
    private const float ArrowLength = 9f;
    private const float FontPixels = 14f;
    private static readonly Vector2 ReadoutOffset = new(30f, 14f);

    private float pixelsPerUnit = 1f;

    /// <summary>The largest torque magnitude in the run, N·m; the arc's full sweep is at this value.</summary>
    public double MaxTorque { get; set; } = 1;

    public string Id { get; set; } = string.Empty;

    /// <summary>The torque the servo applies on the frame on show, N·m.</summary>
    public double Torque { get; private set; }

    public override void _Ready() => SetProcess(true);

    /// <summary>Sets the torque shown and redraws; the position is set by the scene.</summary>
    public void Show(double torque)
    {
        Torque = torque;
        QueueRedraw();
    }

    public override void _Process(double delta)
    {
        // The camera zoom reaches this node through its canvas transform. When it
        // changes, redraw at the new scale so the servo keeps its pixel size.
        var scale = GetGlobalTransformWithCanvas().X.Length();
        if (scale > 0 && Math.Abs(scale - pixelsPerUnit) > 1e-9f)
        {
            pixelsPerUnit = scale;
            QueueRedraw();
        }
    }

    public override void _Draw()
    {
        // Converts a length in screen pixels to this node's local units (run metres).
        float Px(float pixels) => pixels / pixelsPerUnit;

        // The housing: a ring around the joint.
        DrawArc(Vector2.Zero, Px(HousingRadius), 0, Mathf.Tau, 48, Housing, Px(HousingWidth), true);

        // The torque arc, on the top of the joint, sweeping with the torque.
        var arcRadius = Px(ArcRadius);
        var fraction = Mathf.Clamp(Math.Abs(Torque) / MaxTorque, 0, 1);
        if (fraction < 1e-6)
        {
            DrawArc(Vector2.Zero, arcRadius, 0, Mathf.Tau, 48, Faint, Px(ArcWidth), true);
        }
        else
        {
            var sign = Torque >= 0 ? 1f : -1f;
            var sweep = sign * Mathf.DegToRad(200f) * (float)fraction;
            var start = Mathf.Pi / 2;
            var end = start + sweep;
            DrawArc(Vector2.Zero, arcRadius, start, end, 48, Arc, Px(ArcWidth), true);

            // Arrowhead at the end of the arc, pointing the way the torque turns.
            var tip = arcRadius * new Vector2(Mathf.Cos(end), Mathf.Sin(end));
            var travel = sign * new Vector2(-Mathf.Sin(end), Mathf.Cos(end));
            var back = -travel.Normalized() * Px(ArrowLength);
            var side = new Vector2(-back.Y, back.X) * 0.6f;
            DrawColoredPolygon([tip, tip + back + side, tip + back - side], Arc);
        }

        // The readout, in screen orientation: the scene flips y, so the text
        // transform flips it back, and the scale is one font pixel per screen pixel.
        var lines = new[]
        {
            "servo",
            Inv($"τ {Torque:+0.00;-0.00;0.00} N·m"),
            "on ulna",
        };
        var font = ThemeDB.FallbackFont;
        var anchor = new Vector2(Px(ReadoutOffset.X), Px(ReadoutOffset.Y));
        var k = Px(FontPixels) / FontPixels;
        DrawSetTransform(anchor, 0, new Vector2(k, -k));
        for (var i = 0; i < lines.Length; i++)
        {
            DrawString(font, new Vector2(0, i * FontPixels * 1.25f), lines[i], HorizontalAlignment.Left, -1, (int)FontPixels, i == 1 ? Arc : Housing);
        }

        DrawSetTransform(Vector2.Zero, 0, Vector2.One);
    }

    private static string Inv(FormattableString text) => text.ToString(CultureInfo.InvariantCulture);
}
