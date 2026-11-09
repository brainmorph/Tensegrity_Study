using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// A cable drawn between its recorded world endpoints, coloured by its
/// recorded tension. Its own transform stays identity; slack cables
/// (recorded tension 0) are dashed.
/// </summary>
public partial class CableView : Node2D
{
    public string Id { get; set; } = string.Empty;

    /// <summary>Top of the run's tension colour scale, N.</summary>
    public double TensionScale { get; set; }

    /// <summary>Line width, m.</summary>
    public float Stroke { get; set; } = 0.005f;

    /// <summary>Recorded world endpoint A on show.</summary>
    public Vector2 A { get; private set; }

    /// <summary>Recorded world endpoint B on show.</summary>
    public Vector2 B { get; private set; }

    /// <summary>Recorded tension on show, N.</summary>
    public float Tension { get; private set; }

    public bool IsSlack => Tension <= 0;

    public void Show(Vector2 a, Vector2 b, float tension)
    {
        A = a;
        B = b;
        Tension = tension;
        QueueRedraw();
    }

    public override void _Draw()
    {
        if (IsSlack)
        {
            DrawDashedLine(A, B, TensionPalette.Slack, Stroke * 0.8f, Stroke * 3, aligned: true);
        }
        else
        {
            DrawLine(A, B, TensionPalette.For(Tension, TensionScale), Stroke);
        }

        DrawCircle(A, Stroke * 0.9f, TensionPalette.Anchor, filled: true);
        DrawCircle(B, Stroke * 0.9f, TensionPalette.Anchor, filled: true);
    }
}
