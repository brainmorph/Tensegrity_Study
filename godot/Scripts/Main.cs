using Godot;
using Tensegrity.RunFormat;

namespace Tensegrity.Viewer;

/// <summary>
/// Entry point and playback. Opens a run from the command line or the
/// folder picker and shows one recorded frame at a time: during playback
/// the frame on show is the last one recorded at or before the playback
/// clock. Frames are never interpolated.
/// </summary>
public partial class Main : Node
{
    private RunReader? run;
    private RunScene? scene;
    private float[] times = [];
    private double clock;
    private bool playing;
    private double speed = Hud.Speeds[Hud.DefaultSpeedIndex];
    private ViewCamera camera = null!;
    private Hud hud = null!;
    private FileDialog picker = null!;

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

        if (options.FidelityDump)
        {
            GD.PrintErr("--fidelity-dump is not available yet.");
            GetTree().Quit(2);
            return;
        }

        RenderingServer.SetDefaultClearColor(new Color("16181d"));
        camera = new ViewCamera();
        AddChild(camera);
        camera.MakeCurrent();

        hud = new Hud();
        AddChild(hud);
        camera.FreeArea = hud.FreeArea;
        hud.OpenRequested += ShowPicker;
        hud.PlayToggled += TogglePlay;
        hud.StepRequested += Step;
        hud.FrameRequested += frame => { Pause(); ShowFrame(frame); };
        hud.SpeedChanged += s => speed = s;

        picker = new FileDialog
        {
            FileMode = FileDialog.FileModeEnum.OpenDir,
            Access = FileDialog.AccessEnum.Filesystem,
            Title = "Open a run directory",
            UseNativeDialog = false,
        };
        AddChild(picker);
        picker.DirSelected += Open;

        if (options.RunDirectory is null)
        {
            CallDeferred(MethodName.ShowPicker);
        }
        else
        {
            Open(options.RunDirectory);
            if (options.Play)
            {
                TogglePlay();
            }
        }
    }

    public override void _Process(double delta)
    {
        if (!playing || run is null)
        {
            return;
        }

        clock += delta * speed;
        var frame = FrameAt(clock);
        if (frame != scene!.Frame)
        {
            ShowFrame(frame);
        }

        if (frame == run.FrameCount - 1)
        {
            Pause();
        }
    }

    public override void _UnhandledInput(InputEvent @event)
    {
        if (@event is not InputEventKey { Pressed: true } key)
        {
            return;
        }

        switch (key.Keycode)
        {
            case Key.Space when !key.Echo:
                TogglePlay();
                break;
            case Key.Left:
                Step(-1);
                break;
            case Key.Right:
                Step(1);
                break;
            case Key.Home:
                Pause();
                ShowFrame(0);
                break;
            case Key.End when run is not null:
                Pause();
                ShowFrame(run.FrameCount - 1);
                break;
            case Key.F when !key.Echo:
                camera.Refit();
                break;
            case Key.O when !key.Echo:
                ShowPicker();
                break;
            default:
                return;
        }

        GetViewport().SetInputAsHandled();
    }

    /// <summary>Opens a run directory, replacing the run on show.</summary>
    public void Open(string directory)
    {
        RunReader reader;
        var next = new RunScene { Name = "Run" };
        try
        {
            reader = RunReader.Open(directory);
            if (reader.FrameCount == 0)
            {
                throw new InvalidDataException("The run has no frames.");
            }

            next.Build(reader);
        }
        catch (Exception e) when (IsRunError(e))
        {
            next.Free();
            hud.ShowError($"Cannot open {directory}:\n{e.Message}");
            return;
        }

        Pause();
        scene?.QueueFree();
        scene = next;
        AddChild(scene);
        MoveChild(scene, 0);
        run = reader;
        times = reader.Channel(FrameChannels.Time);
        picker.CurrentDir = Path.GetDirectoryName(directory.TrimEnd('/')) ?? directory;
        hud.ShowRun(directory, reader, scene.TensionScale);
        camera.Fit(scene.Bounds);

        // The overlays size themselves on the next layout pass; fit again then.
        Callable.From(camera.Refit).CallDeferred();
        ShowFrame(0);
    }

    private static bool IsRunError(Exception e) =>
        e is IOException or InvalidDataException or UnauthorizedAccessException
            or KeyNotFoundException or System.Text.Json.JsonException;

    private void ShowPicker()
    {
        if (string.IsNullOrEmpty(picker.CurrentDir) || picker.CurrentDir == "/")
        {
            var runs = Path.GetFullPath(Path.Combine(ProjectSettings.GlobalizePath("res://"), "..", "runs"));
            if (Directory.Exists(runs))
            {
                picker.CurrentDir = runs;
            }
        }

        picker.PopupCentered(new Vector2I(900, 600));
    }

    private void ShowFrame(int frame)
    {
        if (run is null)
        {
            return;
        }

        frame = Math.Clamp(frame, 0, run.FrameCount - 1);
        scene!.ShowFrame(frame);
        clock = times[frame];
        hud.ShowFrame(frame, run.FrameCount, times[frame]);
    }

    private void Step(int by)
    {
        if (scene is null)
        {
            return;
        }

        Pause();
        ShowFrame(scene.Frame + by);
    }

    private void TogglePlay()
    {
        if (run is null)
        {
            return;
        }

        if (playing)
        {
            Pause();
            return;
        }

        if (scene!.Frame >= run.FrameCount - 1)
        {
            ShowFrame(0);
        }

        playing = true;
        hud.ShowPlaying(true);
    }

    private void Pause()
    {
        playing = false;
        hud.ShowPlaying(false);
    }

    /// <summary>The last frame recorded at or before run time <paramref name="t"/>.</summary>
    private int FrameAt(double t)
    {
        int lo = 0, hi = times.Length - 1;
        while (lo < hi)
        {
            var mid = (lo + hi + 1) / 2;
            if (times[mid] <= t)
            {
                lo = mid;
            }
            else
            {
                hi = mid - 1;
            }
        }

        return lo;
    }
}
