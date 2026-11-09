using Godot;

namespace Tensegrity.Viewer;

/// <summary>View control only: fit to the run, drag to pan, wheel to zoom about the cursor.</summary>
public partial class ViewCamera : Camera2D
{
    private const float FitMargin = 0.85f;
    private const float WheelStep = 1.15f;
    private Rect2 fitRect = new(-1, -1, 2, 2);
    private bool dragging;

    /// <summary>
    /// The screen region left free by the overlays, in pixels; a fit
    /// centres the run there. Null uses the whole viewport.
    /// </summary>
    public Func<Rect2>? FreeArea { get; set; }

    /// <summary>Frames a run-space (y-up) box.</summary>
    public void Fit(Rect2 runBounds)
    {
        // RunScene flips y, so the box's top edge lands at -End.Y on screen.
        fitRect = new Rect2(runBounds.Position.X, -runBounds.End.Y, runBounds.Size);
        Refit();
    }

    public void Refit()
    {
        var viewport = GetViewportRect().Size;
        var area = FreeArea?.Invoke() ?? new Rect2(Vector2.Zero, viewport);
        if (area.Size.X < 50 || area.Size.Y < 50)
        {
            area = new Rect2(Vector2.Zero, viewport);
        }

        var size = fitRect.Size.Max(new Vector2(1e-3f, 1e-3f));
        var zoom = Mathf.Min(area.Size.X / size.X, area.Size.Y / size.Y) * FitMargin;
        Zoom = new Vector2(zoom, zoom);
        Position = fitRect.GetCenter() - ((area.GetCenter() - (viewport / 2)) / zoom);
    }

    public override void _UnhandledInput(InputEvent @event)
    {
        switch (@event)
        {
            case InputEventMouseButton { ButtonIndex: MouseButton.WheelUp, Pressed: true } wheel:
                ZoomAbout(wheel.Position, WheelStep);
                break;
            case InputEventMouseButton { ButtonIndex: MouseButton.WheelDown, Pressed: true } wheel:
                ZoomAbout(wheel.Position, 1 / WheelStep);
                break;
            case InputEventMouseButton { ButtonIndex: MouseButton.Left or MouseButton.Middle or MouseButton.Right } button:
                dragging = button.Pressed;
                break;
            case InputEventMouseMotion motion when dragging:
                Position -= motion.Relative / Zoom;
                break;
            default:
                return;
        }

        GetViewport().SetInputAsHandled();
    }

    private void ZoomAbout(Vector2 screenPoint, float factor)
    {
        var offset = screenPoint - (GetViewportRect().Size / 2);
        var anchored = Position + (offset / Zoom);
        Zoom *= factor;
        Position = anchored - (offset / Zoom);
    }
}
