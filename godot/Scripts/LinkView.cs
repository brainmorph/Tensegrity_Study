using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A rigid link: a bar along its body x axis, centered on its recorded center of mass.
/// Its label gives its recorded angle, in degrees, just above the bar.
/// </summary>
public partial class LinkView : ScreenScaled
{
    private static readonly Color LinkColor = new("d8dce3");
    private static readonly Color StrutColor = new("f0e6c8");
    private static readonly Color AngleColor = new("9ece6a");

    public string Id { get; set; } = string.Empty;

    /// <summary>Bar length from the topology, m.</summary>
    public float Length { get; set; }

    public bool IsStrut { get; set; }

    /// <summary>Bar width, m.</summary>
    public float Stroke { get; set; } = 0.01f;

    /// <summary>The recorded angle of the bar on show, rad.</summary>
    public float Angle { get; private set; }

    /// <summary>Sets the angle shown and redraws; the position and rotation are set by the scene.</summary>
    public void ShowAngle(float angle)
    {
        Angle = angle;
        QueueRedraw();
    }

    public override void _Draw()
    {
        var half = new Vector2(Length / 2, 0);
        var color = IsStrut ? StrutColor : LinkColor;
        var width = IsStrut ? Stroke * 1.25f : Stroke;
        DrawLine(-half, half, color, width);
        DrawCircle(-half, width / 2, color, filled: true);
        DrawCircle(half, width / 2, color, filled: true);

        var degrees = Mathf.RadToDeg(Angle);
        DrawUpright(new Vector2(0, Px(12f)), $"{Id} {Num(degrees, "0.0")}°", AngleColor, 13);
    }
}
