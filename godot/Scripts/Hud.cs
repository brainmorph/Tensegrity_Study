using System.Globalization;
using Godot;
using Tensegrity.RunFormat;

namespace Tensegrity.Viewer;

/// <summary>
/// On-screen controls and run information: a timeline bar along the
/// bottom and a panel of recorded run facts at the top left. Raises
/// events; the playback state lives in <see cref="Main"/>.
/// </summary>
public partial class Hud : CanvasLayer
{
    /// <summary>Playback speeds, in run seconds per real second.</summary>
    public static readonly double[] Speeds = [0.05, 0.1, 0.25, 0.5, 1, 2];
    public const int DefaultSpeedIndex = 4;

    private static readonly Color Warning = new("ff6b5a");

    private readonly Button open = Tool("Open…");
    private readonly Button play = Tool("Play");
    private readonly Button back = Tool("◀");
    private readonly Button forward = Tool("▶");
    private readonly HSlider slider = new();
    private readonly OptionButton speed = new();
    private readonly Label position = new();
    private readonly Label title = new();
    private readonly Label details = new();
    private readonly Label alerts = new();
    private readonly Label scaleMax = new();
    private readonly Label error = new();
    private readonly PanelContainer info = new();
    private readonly PanelContainer timeline = new();

    public event Action? OpenRequested;

    public event Action? PlayToggled;

    public event Action<int>? StepRequested;

    public event Action<int>? FrameRequested;

    public event Action<double>? SpeedChanged;

    public override void _Ready()
    {
        var root = new Control { MouseFilter = Control.MouseFilterEnum.Ignore };
        AddChild(root);
        root.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.FullRect);

        root.AddChild(BuildTimeline());
        root.AddChild(BuildInfo());

        error.AddThemeColorOverride("font_color", Warning);
        error.AutowrapMode = TextServer.AutowrapMode.WordSmart;
        root.AddChild(error);
        error.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.Center);
        error.HorizontalAlignment = HorizontalAlignment.Center;
        error.CustomMinimumSize = new Vector2(600, 0);
        error.GrowHorizontal = Control.GrowDirection.Both;
        error.GrowVertical = Control.GrowDirection.Both;

        ShowNoRun();
    }

    /// <summary>Shows an open run's facts, all read from its manifest.</summary>
    public void ShowRun(string directory, RunReader run, double tensionScale)
    {
        var m = run.Manifest;
        var d = m.Diagnostics;
        title.Text = $"{m.Topology.Name}   ·   {Path.GetFileName(directory.TrimEnd('/'))}";
        details.Text = string.Join('\n',
            Inv($"{run.FrameCount} frames  ·  dt {m.Dt:G3} s  ·  every {m.FrameStride} steps  ·  {m.Duration:G4} s"),
            Inv($"links {m.Topology.Links.Count}  ·  cables {m.Topology.Cables.Count}  ·  pins {m.Topology.Pins.Count}  ·  gravity ({m.Gravity[0]:G4}, {m.Gravity[1]:G4}) m/s²"),
            Inv($"energy error {d.MaxEnergyBalanceError:G3} J  ·  pin residual {d.MaxPinPositionResidual:G3} m  ·  events {d.EventCount}"),
            $"sim {m.SimVersion}{(m.SimDirty == true ? " (dirty)" : string.Empty)}  ·  {m.CreatedUtc}");
        var warnings = new List<string>();
        if (d.StrutsInTension.Count > 0)
        {
            warnings.Add("struts in tension: " + string.Join(", ", d.StrutsInTension));
        }

        if (d.UnresolvedEventSteps > 0)
        {
            warnings.Add($"unresolved event steps: {d.UnresolvedEventSteps}");
        }

        alerts.Text = string.Join('\n', warnings);
        alerts.Visible = warnings.Count > 0;
        scaleMax.Text = Inv($"{tensionScale:G4} N");

        slider.MaxValue = Math.Max(run.FrameCount - 1, 0);
        info.Visible = true;
        error.Text = string.Empty;
        SetControlsEnabled(true);
    }

    /// <summary>The screen region between the info panel and the timeline bar, in pixels.</summary>
    public Rect2 FreeArea()
    {
        var viewport = GetViewport().GetVisibleRect().Size;
        var top = info.Visible ? info.GetRect().End.Y : 0;
        var bottom = timeline.GetRect().Position.Y;
        return new Rect2(0, top, viewport.X, bottom - top);
    }

    public void ShowNoRun()
    {
        info.Visible = false;
        position.Text = "no run open";
        SetControlsEnabled(false);
    }

    public void ShowError(string message) => error.Text = message;

    /// <summary>Moves the slider and readout to a frame without raising <see cref="FrameRequested"/>.</summary>
    public void ShowFrame(int frame, int frameCount, float time)
    {
        slider.SetValueNoSignal(frame);
        position.Text = Inv($"frame {frame} / {frameCount - 1}   t = {time:F4} s");
    }

    public void ShowPlaying(bool playing) => play.Text = playing ? "Pause" : "Play";

    private static Button Tool(string text) => new() { Text = text, FocusMode = Control.FocusModeEnum.None };

    private static string Inv(FormattableString text) => text.ToString(CultureInfo.InvariantCulture);

    private static StyleBoxFlat Panel() => new()
    {
        BgColor = new Color(0.07f, 0.08f, 0.1f, 0.88f),
        ContentMarginLeft = 12,
        ContentMarginRight = 12,
        ContentMarginTop = 8,
        ContentMarginBottom = 8,
    };

    private static StyleBoxFlat Track(Color color) => new()
    {
        BgColor = color,
        ContentMarginTop = 3,
        ContentMarginBottom = 3,
        CornerRadiusTopLeft = 3,
        CornerRadiusTopRight = 3,
        CornerRadiusBottomLeft = 3,
        CornerRadiusBottomRight = 3,
    };

    private Control BuildTimeline()
    {
        var bar = timeline;
        bar.AddThemeStyleboxOverride("panel", Panel());
        var row = new HBoxContainer();
        row.AddThemeConstantOverride("separation", 8);
        bar.AddChild(row);

        slider.Step = 1;
        slider.MinValue = 0;
        slider.FocusMode = Control.FocusModeEnum.None;
        slider.SizeFlagsHorizontal = Control.SizeFlags.ExpandFill;
        slider.SizeFlagsVertical = Control.SizeFlags.ShrinkCenter;
        slider.AddThemeStyleboxOverride("slider", Track(new Color("3a3f48")));
        slider.AddThemeStyleboxOverride("grabber_area", Track(new Color("7aa2f7")));
        slider.AddThemeStyleboxOverride("grabber_area_highlight", Track(new Color("9bbcff")));

        foreach (var s in Speeds)
        {
            speed.AddItem(string.Create(CultureInfo.InvariantCulture, $"{s:G}×"));
        }

        speed.Selected = DefaultSpeedIndex;
        speed.FocusMode = Control.FocusModeEnum.None;
        speed.TooltipText = "Playback speed: run seconds per real second";
        position.CustomMinimumSize = new Vector2(230, 0);

        open.Pressed += () => OpenRequested?.Invoke();
        play.Pressed += () => PlayToggled?.Invoke();
        back.Pressed += () => StepRequested?.Invoke(-1);
        forward.Pressed += () => StepRequested?.Invoke(1);
        slider.ValueChanged += value => FrameRequested?.Invoke((int)value);
        speed.ItemSelected += index => SpeedChanged?.Invoke(Speeds[index]);

        foreach (var control in new Control[] { open, play, back, forward, slider, speed, position })
        {
            row.AddChild(control);
        }

        bar.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.BottomWide);
        bar.GrowVertical = Control.GrowDirection.Begin;
        return bar;
    }

    private Control BuildInfo()
    {
        var column = new VBoxContainer();
        info.AddThemeStyleboxOverride("panel", Panel());
        info.AddChild(column);
        title.AddThemeFontSizeOverride("font_size", 18);
        alerts.AddThemeColorOverride("font_color", Warning);
        details.AddThemeColorOverride("font_color", new Color("b8bec8"));

        var legend = new HBoxContainer();
        legend.AddThemeConstantOverride("separation", 6);
        var bar = new GradientBar { CustomMinimumSize = new Vector2(160, 10), SizeFlagsVertical = Control.SizeFlags.ShrinkCenter };
        var slack = new Label { Text = "   slack: dashed", Modulate = TensionPalette.Slack.Lightened(0.3f) };
        foreach (var control in new Control[] { new Label { Text = "tension  0 N" }, bar, scaleMax, slack })
        {
            legend.AddChild(control);
        }

        foreach (var control in new Control[] { title, details, alerts, legend })
        {
            column.AddChild(control);
        }

        info.Position = new Vector2(12, 12);
        return info;
    }

    private void SetControlsEnabled(bool enabled)
    {
        play.Disabled = !enabled;
        back.Disabled = !enabled;
        forward.Disabled = !enabled;
        slider.Editable = enabled;
    }
}
