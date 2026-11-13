using System.Globalization;
using Godot;

namespace Tensegrity.Viewer;

/// <summary>
/// Render-fidelity output (PLAN.md section 7, test 16). Shows every frame
/// in the same <see cref="RunScene"/> the viewer draws, and prints the
/// transform and drawn values each node then holds. The test compares
/// them with the recorded frames, so what is checked is what is drawn.
/// Prints only; reads and writes no files.
/// </summary>
public static class FidelityDump
{
    public const string Prefix = "FIDELITY";

    public static void Print(RunScene scene, int frameCount)
    {
        var t = scene.Transform;
        // Counts the structure only (links, cables, pins), not the torque and force views drawn over it.
        var structure = scene.Links.Count() + scene.Cables.Count() + scene.Pins.Count();
        Line("scene", F(t.X.X), F(t.X.Y), F(t.Y.X), F(t.Y.Y), F(t.Origin.X), F(t.Origin.Y), structure.ToString(CultureInfo.InvariantCulture));
        for (var frame = 0; frame < frameCount; frame++)
        {
            scene.ShowFrame(frame);
            var f = frame.ToString(CultureInfo.InvariantCulture);
            foreach (var link in scene.Links)
            {
                Line("link", f, link.Id, F(link.Position.X), F(link.Position.Y), F(link.Rotation), F(link.Scale.X), F(link.Scale.Y), F(link.Skew));
            }

            foreach (var cable in scene.Cables)
            {
                Line("cable", f, cable.Id, F(cable.A.X), F(cable.A.Y), F(cable.B.X), F(cable.B.Y), F(cable.Tension), F(cable.Position.X), F(cable.Position.Y), F(cable.Rotation));
            }

            foreach (var pin in scene.Pins)
            {
                Line("pin", f, pin.Id, F(pin.Position.X), F(pin.Position.Y), F(pin.Rotation));
            }
        }

        Line("end", frameCount.ToString(CultureInfo.InvariantCulture));
    }

    private static string F(float value) => value.ToString("R", CultureInfo.InvariantCulture);

    private static void Line(params string[] fields) => GD.Print(Prefix + " " + string.Join(' ', fields));
}
