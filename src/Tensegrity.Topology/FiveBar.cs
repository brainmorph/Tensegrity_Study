using Tensegrity.Core.Analysis;
using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Topology;

/// <summary>
/// The tendon-held five-bar of <see cref="FiveBarGeometry"/> as a
/// simulator model, posed at a configuration from
/// <see cref="FiveBarStatics"/>. Links run A→C (the servo link), C→E, D→E
/// and B→D (the passive link), each with end A at the first point. The
/// servo pin's end A is the servo link, so a positive holding torque there
/// turns it counter-clockwise, the statics' sign. No actuator is attached:
/// the model is the shape alone, released at rest.
/// </summary>
public static class FiveBar
{
    public const string ServoLink = "servo_link";
    public const string CouplerC = "coupler_c";
    public const string CouplerD = "coupler_d";
    public const string PassiveLink = "passive_link";
    public const string ServoPin = "servo";
    public const string HingeC = "c";
    public const string HingeE = "e";
    public const string HingeD = "d";
    public const string PassivePin = "passive";
    public const string TendonC = "tendon_c";
    public const string TendonD = "tendon_d";
    public const string TendonE = "tendon_e";
    public const string TendonCE = "tendon_ce";

    /// <summary>The five-bar at rest in <paramref name="pose"/>.</summary>
    public static PlanarModel Create(FiveBarGeometry geometry, FiveBarPose pose, string name = "five_bar", string? description = null, SimulationDocument? simulation = null)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        geometry.Validate();
        var servo = PlanarLink.SlenderBar(ServoLink, geometry.LinkMass, geometry.ServoLength);
        var couplerC = PlanarLink.SlenderBar(CouplerC, geometry.LinkMass, geometry.CouplerCLength);
        var couplerD = PlanarLink.SlenderBar(CouplerD, geometry.LinkMass, geometry.CouplerDLength);
        var passive = PlanarLink.SlenderBar(PassiveLink, geometry.LinkMass, geometry.PassiveLength);
        var a = Vector2d.Zero;
        var b = new Vector2d(geometry.BaseWidth, 0);

        PinJoint[] pins =
        [
            new(ServoPin, Attachment.OnLink(ServoLink, servo.EndA), Attachment.Ground(a)),
            new(HingeC, Attachment.OnLink(CouplerC, couplerC.EndA), Attachment.OnLink(ServoLink, servo.EndB)),
            new(HingeE, Attachment.OnLink(CouplerD, couplerD.EndB), Attachment.OnLink(CouplerC, couplerC.EndB)),
            new(HingeD, Attachment.OnLink(CouplerD, couplerD.EndA), Attachment.OnLink(PassiveLink, passive.EndB)),
            new(PassivePin, Attachment.OnLink(PassiveLink, passive.EndA), Attachment.Ground(b)),
        ];

        Cable Tendon(string id, FiveBarTendon tendon, Attachment hinge) =>
            new(id, Attachment.Ground(tendon.Anchor), hinge, tendon.RestLength, geometry.AxialStiffness, geometry.TendonDamping);

        Cable[] cables =
        [
            Tendon(TendonC, geometry.TendonC, Attachment.OnLink(ServoLink, servo.EndB)),
            Tendon(TendonD, geometry.TendonD, Attachment.OnLink(PassiveLink, passive.EndB)),
            Tendon(TendonE, geometry.TendonE, Attachment.OnLink(CouplerC, couplerC.EndB)),
            Tendon(TendonCE, geometry.TendonCE, Attachment.OnLink(CouplerC, couplerC.EndA + new Vector2d(geometry.TendonCEPosition * geometry.CouplerCLength, 0))),
        ];

        var system = new PlanarSystem([servo, couplerC, couplerD, passive], new Vector2d(0, -geometry.Gravity), pins, cables);
        var state = system.CreateState().Values;
        Pose2d Place(Vector2d start, double length, double angle) => new(start + (length / 2 * new Vector2d(Math.Cos(angle), Math.Sin(angle))), angle);
        var c = a + (geometry.ServoLength * new Vector2d(Math.Cos(pose.Servo), Math.Sin(pose.Servo)));
        var d = b + (geometry.PassiveLength * new Vector2d(Math.Cos(pose.Passive), Math.Sin(pose.Passive)));
        system.SetBody(state, 0, Place(a, geometry.ServoLength, pose.Servo), default);
        system.SetBody(state, 1, Place(c, geometry.CouplerCLength, pose.CouplerC), default);
        system.SetBody(state, 2, Place(d, geometry.CouplerDLength, pose.CouplerD), default);
        system.SetBody(state, 3, Place(b, geometry.PassiveLength, pose.Passive), default);
        return new PlanarModel(name, system, state, description, simulation);
    }
}
