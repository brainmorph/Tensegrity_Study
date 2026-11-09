using Godot;
using Tensegrity.RunFormat;

namespace Tensegrity.Viewer;

/// <summary>Entry point: parses the command line and opens a run.</summary>
public partial class Main : Node
{
    public override void _Ready()
    {
        ViewerOptions options;
        try
        {
            options = ViewerOptions.Parse(OS.GetCmdlineUserArgs(), ViewerOptions.LaunchDirectory());
        }
        catch (ArgumentException e)
        {
            GD.PrintErr(e.Message);
            GetTree().Quit(2);
            return;
        }

        if (options.RunDirectory is null)
        {
            GD.Print("No run given. ", ViewerOptions.Usage);
            return;
        }

        try
        {
            var run = RunReader.Open(options.RunDirectory);
            GD.Print($"Opened {options.RunDirectory}: {run.Manifest.Topology.Name}, {run.FrameCount} frames.");
        }
        catch (Exception e) when (e is IOException or InvalidDataException or System.Text.Json.JsonException or UnauthorizedAccessException)
        {
            GD.PrintErr($"Cannot open run {options.RunDirectory}: {e.Message}");
            GetTree().Quit(1);
        }
    }
}
