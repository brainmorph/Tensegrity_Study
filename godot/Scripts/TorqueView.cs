using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A torque applied at a pin: a ring around the joint, an arc whose sweep and
/// arrowhead show the recorded torque, and a readout beside it. Used for servos
/// (actuators) and for joint limits. +τ acts on the pin's end A
/// (counter-clockwise in run space), −τ on end B.
/// </summary>
public partial class TorqueView : ScreenScaled
{
    // Sizes in screen pixels.
    private const float RingRadius = 14f;
    private const float RingWidth = 3.5f;
    private const float ArrowLength = 9f;
    private const float ArcWidth = 3.5f;

    /// <summary>Arc radius, screen pixels. Stacked torques use different radii.</summary>
    public float ArcRadiusPx { get; set; } = 26f;

    /// <summary>Readout anchor, screen pixels from the pin, y up.</summary>
    public Vector2 ReadoutOffsetPx { get; set; } = new(30f, 14f);

    /// <summary>The colour of the ring's arc and readout.</summary>
    public Color Tint { get; set; } = new("ffd27a");

    /// <summary>The first readout line, e.g. "servo".</summary>
    public string Caption { get; set; } = string.Empty;

    /// <summary>The last readout line, e.g. "on ulna".</summary>
    public string Note { get; set; } = string.Empty;

    /// <summary>The largest torque magnitude in the run, N·m; the arc's full sweep is at this value.</summary>
    public double MaxTorque { get; set; } = 1;

    /// <summary>The torque applied on the frame on show, N·m.</summary>
    public double Torque { get; private set; }

    /// <summary>Sets the torque shown and redraws; the position is set by the scene.</summary>
    public void Show(double torque)
    {
        Torque = torque;
        QueueRedraw();
    }

    public override void _Draw()
    {
        var ringColor = Tint;
        DrawArc(Vector2.Zero, Px(RingRadius), 0, Mathf.Tau, 48, ringColor, Px(RingWidth), true);

        var arcRadius = Px(ArcRadiusPx);
        var fraction = Mathf.Clamp(Math.Abs(Torque) / MaxTorque, 0, 1);
        if (fraction < 1e-6)
        {
            DrawArc(Vector2.Zero, arcRadius, 0, Mathf.Tau, 48, new Color(Tint.R, Tint.G, Tint.B, 0.35f), Px(ArcWidth), true);
        }
        else
        {
            var sign = Torque >= 0 ? 1f : -1f;
            var sweep = sign * Mathf.DegToRad(200f) * (float)fraction;
            var start = Mathf.Pi / 2;
            var end = start + sweep;
            DrawArc(Vector2.Zero, arcRadius, start, end, 48, Tint, Px(ArcWidth), true);

            // Arrowhead at the end of the arc, pointing the way the torque turns.
            var tip = arcRadius * new Vector2(Mathf.Cos(end), Mathf.Sin(end));
            var travel = sign * new Vector2(-Mathf.Sin(end), Mathf.Cos(end));
            var back = -travel.Normalized() * Px(ArrowLength);
            var side = new Vector2(-back.Y, back.X) * 0.6f;
            DrawColoredPolygon([tip, tip + back + side, tip + back - side], Tint);
        }

        var readout = new[]
        {
            Caption,
            $"τ {Num(Torque, "+0.00;-0.00;0.00")} N·m",
            Note,
        };
        for (var i = 0; i < readout.Length; i++)
        {
            var anchor = new Vector2(Px(ReadoutOffsetPx.X), Px(ReadoutOffsetPx.Y - i * 16f));
            DrawUpright(anchor, readout[i], i == 1 ? Tint : new Color("c8ccd4"), 14);
        }
    }
}
