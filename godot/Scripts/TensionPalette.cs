using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// Maps a recorded cable tension to a colour on one scale per run: 0 N to
/// the highest tension the core recorded for any cable in the run.
/// </summary>
public static class TensionPalette
{
    public static readonly Color Low = new("4c8bf5");
    public static readonly Color Mid = new("f5c542");
    public static readonly Color High = new("e5482e");
    public static readonly Color Slack = new("6b7280");
    public static readonly Color Anchor = new("c9ced6");

    /// <summary>Colour at <paramref name="fraction"/> of the scale, 0 to 1.</summary>
    public static Color At(float fraction)
    {
        var t = Mathf.Clamp(fraction, 0f, 1f);
        return t < 0.5f ? Low.Lerp(Mid, t * 2) : Mid.Lerp(High, (t - 0.5f) * 2);
    }

    public static Color For(float tension, double scale) =>
        scale > 0 ? At((float)(tension / scale)) : Low;
}
