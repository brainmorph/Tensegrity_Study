using Godot;

namespace Tensegrity.Viewer;

/// <summary>The tension colour scale, drawn left (0 N) to right (run maximum).</summary>
public partial class GradientBar : Control
{
    private const int Slices = 48;

    public override void _Draw()
    {
        var width = Size.X / Slices;
        for (var i = 0; i < Slices; i++)
        {
            DrawRect(new Rect2(i * width, 0, width + 0.5f, Size.Y), TensionPalette.At((i + 0.5f) / Slices));
        }
    }
}
