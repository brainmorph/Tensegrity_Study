using Godot;

namespace Tensegrity.Viewer;

/// <summary>A rigid link: a bar along its body x axis, centered on its recorded center of mass.</summary>
public partial class LinkView : Node2D
{
    private static readonly Color LinkColor = new("d8dce3");
    private static readonly Color StrutColor = new("f0e6c8");

    public string Id { get; set; } = string.Empty;

    /// <summary>Bar length from the topology, m.</summary>
    public float Length { get; set; }

    public bool IsStrut { get; set; }

    /// <summary>Bar width, m.</summary>
    public float Stroke { get; set; } = 0.01f;

    public override void _Draw()
    {
        var half = new Vector2(Length / 2, 0);
        var color = IsStrut ? StrutColor : LinkColor;
        var width = IsStrut ? Stroke * 1.25f : Stroke;
        DrawLine(-half, half, color, width);
        DrawCircle(-half, width / 2, color, filled: true);
        DrawCircle(half, width / 2, color, filled: true);
    }
}
