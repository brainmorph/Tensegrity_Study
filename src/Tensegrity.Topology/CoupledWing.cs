using Tensegrity.Core.Analysis;
using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// The one-input coupled wing of <see cref="CoupledWingGeometry"/> as a
/// simulator model (brainstorm-coupled-wing.md). The upper arm is the
/// ground frame, so it is not a body: the elbow and the radius's root are
/// pins to the ground. Three bodies (ulna, radius, hand) and four pins
/// leave one degree of freedom, the elbow angle, which a servo actuator
/// drives. The elbow has open and folded stops, and an elastic ligament
/// crossing the elbow presses it into whichever stop is nearer; the rest
/// length and stop angles come from <see cref="CoupledWingStatics"/>, so
/// the open and folded poses are exact equilibria with the servo idle.
/// </summary>
public static class CoupledWing
{
    public const string Ulna = "ulna";
    public const string Radius = "radius";
    public const string Hand = "hand";
    public const string Elbow = "elbow";
    public const string RadiusRoot = "radius_root";
    public const string Wrist = "wrist";
    public const string RadiusTip = "radius_tip";
    public const string Ligament = "ligament";
    public const string Servo = "servo";

    /// <summary>Id of a gust at the wingtip.</summary>
    public const string Gust = "gust";

    /// <summary>
    /// The wing at rest at elbow angle <paramref name="elbow"/> (the open
    /// pose if omitted), with the servo following
    /// <paramref name="servo"/> (idle if omitted) and optional
    /// <paramref name="loads"/>.
    /// </summary>
    public static PlanarModel Create(
        CoupledWingGeometry? geometry = null,
        double? elbow = null,
        Schedule? servo = null,
        IEnumerable<ExternalLoad>? loads = null)
    {
        var g = geometry ?? new CoupledWingGeometry();
        var statics = new CoupledWingStatics(g);
        var ulna = PlanarLink.SlenderBar(Ulna, g.UlnaMass, g.UlnaLength);
        var radius = PlanarLink.SlenderBar(Radius, g.RadiusMass, g.RadiusLength);
        var hand = PlanarLink.SlenderBar(Hand, g.HandMass, g.HandLength);
        var offset = g.RadiusOffset * new Vector2d(Math.Cos(g.RadiusOffsetAngle), Math.Sin(g.RadiusOffsetAngle));

        PinJoint[] pins =
        [
            new(Elbow, Attachment.OnLink(Ulna, ulna.EndA), Attachment.Ground(g.Elbow), limit: new JointLimit(statics.FoldStop, statics.OpenStop, g.StopStiffness, g.StopDamping)),
            new(RadiusRoot, Attachment.OnLink(Radius, radius.EndA), Attachment.Ground(g.RadiusRoot)),
            new(Wrist, Attachment.OnLink(Hand, hand.EndA), Attachment.OnLink(Ulna, ulna.EndB)),
            new(RadiusTip, Attachment.OnLink(Radius, radius.EndB), Attachment.OnLink(Hand, hand.EndA + offset)),
        ];

        Cable[] cables =
        [
            new(Ligament, Attachment.Ground(g.LigamentAnchor), Attachment.OnLink(Ulna, ulna.EndA + new Vector2d(g.InsertionRadius, 0)), statics.RestLength, g.LigamentStiffness, g.LigamentDamping),
        ];

        JointActuator[] actuators = [new(Servo, Elbow, servo ?? Schedule.Constant(0))];

        var system = new PlanarSystem([ulna, radius, hand], new Vector2d(0, -g.Gravity), pins, cables, actuators: actuators, loads: loads);
        var state = system.CreateState().Values;
        var poses = statics.BodyPoses(elbow ?? g.OpenAngle);
        for (var b = 0; b < 3; b++)
        {
            system.SetBody(state, b, poses[b], default);
        }

        return new PlanarModel("coupled_wing", system, state);
    }

    /// <summary>
    /// A servo schedule that folds the wing and opens it again: −<paramref name="torque"/>
    /// from <paramref name="foldStart"/> for <paramref name="hold"/>, then
    /// +<paramref name="torque"/> from <paramref name="openStart"/> for the
    /// same time, each ramped on and off over <paramref name="ramp"/>.
    /// The servo pushes only long enough to carry the elbow past the
    /// over-center angle; the ligament does the rest.
    /// </summary>
    public static Schedule FoldAndOpen(double torque, double foldStart, double openStart, double hold, double ramp)
    {
        IEnumerable<SchedulePoint> Pulse(double start, double value) =>
        [
            new(start, 0), new(start + ramp, value), new(start + ramp + hold, value), new(start + (2 * ramp) + hold, 0),
        ];

        return new Schedule([.. Pulse(foldStart, -torque), .. Pulse(openStart, torque)]);
    }

    /// <summary>A downward gust at the wingtip: a triangular pulse of <paramref name="impulse"/> N·s.</summary>
    public static ExternalLoad WingtipGust(double impulse, double start, double duration, CoupledWingGeometry? geometry = null)
    {
        var g = geometry ?? new CoupledWingGeometry();
        return ExternalLoad.Pulse(Gust, Hand, new Vector2d(g.HandLength / 2, 0), new Vector2d(0, -impulse), start, duration);
    }
}
