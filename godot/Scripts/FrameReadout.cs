using System.Globalization;
using System.Text;
using Godot;
using Tensegrity.RunFormat;

namespace Tensegrity.Viewer;

/// <summary>
/// The text panel for a frame: every recorded angle, cable tension, pin force,
/// torque, load and ground-contact force, as stored in the run. Values are
/// shown as recorded, in degrees for angles, newtons for forces and N·m for
/// torques; nothing is derived.
/// </summary>
public static class FrameReadout
{
    public static string Build(RunReader reader, int frame, double time)
    {
        var topology = reader.Manifest.Topology;
        var values = reader.Frame(frame);
        var text = new StringBuilder();

        text.Append(Inv($"t = {time:F4} s   frame {frame} / {reader.FrameCount - 1}")).Append('\n');

        text.Append("\nangles, world (deg)\n");
        foreach (var link in topology.Links)
        {
            var theta = values[reader.ChannelIndex(FrameChannels.LinkTheta(link.Id))];
            text.Append(Inv($"  {link.Id,-12}{Mathf.RadToDeg(theta),9:0.00}\n"));
        }

        if (topology.Cables.Count > 0)
        {
            text.Append("\ncable tension (N)\n");
            foreach (var cable in topology.Cables)
            {
                var tension = values[reader.ChannelIndex(FrameChannels.CableTension(cable.Id))];
                text.Append(Inv($"  {cable.Id,-12}{tension,9:0.00}\n"));
            }
        }

        var pins = topology.Pins.Where(p => reader.HasChannel(FrameChannels.PinFx(p.Id))).ToList();
        if (pins.Count > 0)
        {
            text.Append("\npin forces on end A (N)\n");
            text.Append(Inv($"  {"",-12}{"Fx",9}{"Fy",10}\n"));
            foreach (var pin in pins)
            {
                var fx = values[reader.ChannelIndex(FrameChannels.PinFx(pin.Id))];
                var fy = values[reader.ChannelIndex(FrameChannels.PinFy(pin.Id))];
                text.Append(Inv($"  {pin.Id,-12}{fx,9:0.00}{fy,10:0.00}\n"));
            }
        }

        var limits = topology.Pins.Where(p => reader.HasChannel(FrameChannels.PinLimitTorque(p.Id))).ToList();
        if (reader.Manifest.Actuators.Count > 0 || limits.Count > 0)
        {
            text.Append("\ntorques on end A (N·m)\n");
            foreach (var actuator in reader.Manifest.Actuators)
            {
                var torque = values[reader.ChannelIndex(FrameChannels.ActuatorTorque(actuator.Id))];
                text.Append(Inv($"  servo {actuator.Id,-7}{torque,9:+0.00;-0.00;0.00}\n"));
            }

            foreach (var pin in limits)
            {
                var torque = values[reader.ChannelIndex(FrameChannels.PinLimitTorque(pin.Id))];
                text.Append(Inv($"  limit {pin.Id,-7}{torque,9:+0.00;-0.00;0.00}\n"));
            }
        }

        if (topology.Loads is { Count: > 0 } loads)
        {
            text.Append("\nloads (N)\n");
            foreach (var load in loads)
            {
                if (!reader.HasChannel(FrameChannels.LoadFx(load.Id)))
                {
                    continue;
                }

                var fx = values[reader.ChannelIndex(FrameChannels.LoadFx(load.Id))];
                var fy = values[reader.ChannelIndex(FrameChannels.LoadFy(load.Id))];
                text.Append(Inv($"  {load.Id,-12}{fx,9:0.00}{fy,10:0.00}\n"));
            }
        }

        var contacts = topology.Ground?.Contacts ?? new List<ContactPointDocument>();
        var recorded = contacts.Where(c => reader.HasChannel(FrameChannels.ContactNormal(c.Id))).ToList();
        if (recorded.Count > 0)
        {
            text.Append("\nground contacts (N)\n");
            text.Append(Inv($"  {"",-12}{"normal",9}{"friction",10}\n"));
            foreach (var contact in recorded)
            {
                var normal = values[reader.ChannelIndex(FrameChannels.ContactNormal(contact.Id))];
                var friction = values[reader.ChannelIndex(FrameChannels.ContactFriction(contact.Id))];
                text.Append(Inv($"  {contact.Id,-12}{normal,9:0.00}{friction,10:0.00}\n"));
            }
        }

        return text.ToString().TrimEnd('\n');
    }

    private static string Inv(FormattableString text) => text.ToString(CultureInfo.InvariantCulture);
}
