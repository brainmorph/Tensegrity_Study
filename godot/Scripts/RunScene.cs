using Godot;
using Tensegrity.RunFormat;

namespace Tensegrity.Viewer;

/// <summary>
/// The recorded structure. Its own transform is a fixed y-flip (the run is
/// y-up, the screen y-down) and nothing else, so each child holds exactly
/// the recorded values of the frame on show: links at their recorded
/// center and angle, cables between their recorded endpoints, pins at
/// their recorded positions, and the ground line (if any) at its recorded
/// height. Nothing is interpolated or derived.
/// </summary>
public partial class RunScene : Node2D
{
    /// <summary>The scene's fixed transform: the y-flip and nothing else.</summary>
    public static readonly Transform2D ViewFlip = new(new Vector2(1, 0), new Vector2(0, -1), Vector2.Zero);

    private readonly List<LinkSlot> links = [];
    private readonly List<CableSlot> cables = [];
    private readonly List<PinSlot> pins = [];
    private RunReader? run;
    private int groundHeightChannel = -1;

    /// <summary>The ground line, or null when the run has none.</summary>
    public GroundView? Ground { get; private set; }

    public int Frame { get; private set; } = -1;

    public IEnumerable<LinkView> Links => links.Select(l => l.View);

    public IEnumerable<CableView> Cables => cables.Select(c => c.View);

    public IEnumerable<PinView> Pins => pins.Select(p => p.View);

    /// <summary>
    /// Run-space box (y-up) holding the structure over every frame, padded.
    /// Used only to frame the camera and size line widths.
    /// </summary>
    public Rect2 Bounds { get; private set; }

    /// <summary>The colour scale's top: the highest cable tension the core recorded over the run, N.</summary>
    public double TensionScale { get; private set; }

    /// <summary>Builds one view per link, cable and pin, and shows frame 0.</summary>
    public void Build(RunReader reader)
    {
        run = reader;
        Transform = ViewFlip;
        var topology = reader.Manifest.Topology;
        TensionScale = reader.Manifest.Diagnostics.CableTensions.Select(c => c.MaxTension).DefaultIfEmpty(0).Max();

        // Draw order: ground under cables under links under pins.
        if (topology.Ground is { } ground)
        {
            // Runs from before the ground.height channel have a fixed ground at the topology's height.
            Ground = new GroundView { Name = "ground", Position = new Vector2(0, (float)ground.Height) };
            AddChild(Ground);
            groundHeightChannel = reader.HasChannel(FrameChannels.GroundHeight) ? reader.ChannelIndex(FrameChannels.GroundHeight) : -1;
        }

        foreach (var cable in topology.Cables)
        {
            var view = new CableView { Name = "cable_" + cable.Id, Id = cable.Id, TensionScale = TensionScale };
            AddChild(view);
            cables.Add(new CableSlot(
                view,
                reader.ChannelIndex(FrameChannels.CableAx(cable.Id)),
                reader.ChannelIndex(FrameChannels.CableAy(cable.Id)),
                reader.ChannelIndex(FrameChannels.CableBx(cable.Id)),
                reader.ChannelIndex(FrameChannels.CableBy(cable.Id)),
                reader.ChannelIndex(FrameChannels.CableTension(cable.Id))));
        }

        foreach (var link in topology.Links)
        {
            var view = new LinkView { Name = "link_" + link.Id, Id = link.Id, Length = (float)link.Length, IsStrut = link.Role == LinkRoles.Strut };
            AddChild(view);
            links.Add(new LinkSlot(
                view,
                reader.ChannelIndex(FrameChannels.LinkX(link.Id)),
                reader.ChannelIndex(FrameChannels.LinkY(link.Id)),
                reader.ChannelIndex(FrameChannels.LinkTheta(link.Id))));
        }

        foreach (var pin in topology.Pins)
        {
            var view = new PinView { Name = "pin_" + pin.Id, Id = pin.Id, IsGround = pin.A.IsGround || pin.B.IsGround };
            AddChild(view);
            pins.Add(new PinSlot(
                view,
                reader.ChannelIndex(FrameChannels.PinX(pin.Id)),
                reader.ChannelIndex(FrameChannels.PinY(pin.Id))));
        }

        Bounds = MeasureBounds(reader, topology);
        var stroke = Mathf.Max(Bounds.Size.X, Bounds.Size.Y) * 0.012f;
        foreach (var slot in links)
        {
            slot.View.Stroke = stroke;
        }

        foreach (var slot in cables)
        {
            slot.View.Stroke = stroke * 0.45f;
        }

        foreach (var slot in pins)
        {
            slot.View.Radius = stroke * 0.9f;
        }

        if (Ground is not null)
        {
            // Wide enough to run off the screen whatever the zoom.
            var span = Mathf.Max(Bounds.Size.X, Bounds.Size.Y) * 5;
            Ground.Left = Bounds.GetCenter().X - span;
            Ground.Right = Bounds.GetCenter().X + span;
            Ground.Stroke = stroke;
        }

        ShowFrame(0);
    }

    /// <summary>Sets every view to the recorded values of <paramref name="frame"/>.</summary>
    public void ShowFrame(int frame)
    {
        var values = run!.Frame(frame);
        if (Ground is not null && groundHeightChannel >= 0)
        {
            Ground.Position = new Vector2(0, values[groundHeightChannel]);
        }

        foreach (var slot in links)
        {
            slot.View.Position = new Vector2(values[slot.X], values[slot.Y]);
            slot.View.Rotation = values[slot.Theta];
        }

        foreach (var slot in cables)
        {
            slot.View.Show(new Vector2(values[slot.Ax], values[slot.Ay]), new Vector2(values[slot.Bx], values[slot.By]), values[slot.Tension]);
        }

        foreach (var slot in pins)
        {
            slot.View.Position = new Vector2(values[slot.X], values[slot.Y]);
        }

        Frame = frame;
    }

    private Rect2 MeasureBounds(RunReader reader, TopologyDocument topology)
    {
        var min = new Vector2(float.PositiveInfinity, float.PositiveInfinity);
        var max = new Vector2(float.NegativeInfinity, float.NegativeInfinity);

        void Grow(Vector2 point, float radius)
        {
            min = min.Min(point - new Vector2(radius, radius));
            max = max.Max(point + new Vector2(radius, radius));
        }

        for (var f = 0; f < reader.FrameCount; f++)
        {
            var values = reader.Frame(f);
            for (var i = 0; i < links.Count; i++)
            {
                Grow(new Vector2(values[links[i].X], values[links[i].Y]), (float)topology.Links[i].Length / 2);
            }

            foreach (var slot in cables)
            {
                Grow(new Vector2(values[slot.Ax], values[slot.Ay]), 0);
                Grow(new Vector2(values[slot.Bx], values[slot.By]), 0);
            }

            foreach (var slot in pins)
            {
                Grow(new Vector2(values[slot.X], values[slot.Y]), 0);
            }
        }

        if (topology.Ground is { } ground && float.IsFinite(min.X))
        {
            // Keep the line in view at every height it takes: the camera frames the structure and the ground under it.
            Grow(new Vector2(min.X, (float)ground.Height), 0);
            if (groundHeightChannel >= 0)
            {
                for (var f = 0; f < reader.FrameCount; f++)
                {
                    Grow(new Vector2(min.X, reader.Frame(f)[groundHeightChannel]), 0);
                }
            }
        }

        if (!float.IsFinite(min.X) || !float.IsFinite(max.X))
        {
            return new Rect2(-1, -1, 2, 2);
        }

        var size = max - min;
        var pad = Mathf.Max(Mathf.Max(size.X, size.Y) * 0.1f, 0.05f);
        return new Rect2(min - new Vector2(pad, pad), size + new Vector2(2 * pad, 2 * pad));
    }

    private readonly record struct LinkSlot(LinkView View, int X, int Y, int Theta);

    private readonly record struct CableSlot(CableView View, int Ax, int Ay, int Bx, int By, int Tension);

    private readonly record struct PinSlot(PinView View, int X, int Y);
}
