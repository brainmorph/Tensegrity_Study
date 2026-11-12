using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A rigid link: a bar along its body x axis, centered on its recorded
/// center of mass, with a post out to each cable attachment that sits off
/// the bar. A post is part of the rigid link (a bony process), so the
/// cable end visibly belongs to the link it pulls on.
/// </summary>
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

    /// <summary>Cable attachment points off the bar, in the link's body frame (from the topology), m.</summary>
    public IReadOnlyList<Vector2> Posts { get; set; } = [];

    public override void _Draw()
    {
        var half = new Vector2(Length / 2, 0);
        var color = IsStrut ? StrutColor : LinkColor;
        var width = IsStrut ? Stroke * 1.25f : Stroke;
        DrawLine(-half, half, color, width);
        DrawCircle(-half, width / 2, color, filled: true);
        DrawCircle(half, width / 2, color, filled: true);

        // Each post rises from the nearest point of the bar to its attachment.
        foreach (var post in Posts)
        {
            var foot = new Vector2(Mathf.Clamp(post.X, -half.X, half.X), 0);
            DrawLine(foot, post, color, width * 0.6f);
            DrawCircle(post, width * 0.3f, color, filled: true);
        }
    }
}
