using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// Dimensions of a <see cref="FourBarClamp"/>, SI units. Generic and
/// hand-sized; the links satisfy Grashof's condition (shortest + longest ≤
/// the other two), so the crank can turn all the way round.
/// </summary>
/// <param name="GroundLength">Distance between the crank pivot (origin) and the rocker pivot (+x), m.</param>
/// <param name="CrankLength">Crank (the handle), m.</param>
/// <param name="CouplerLength">Coupler, crank to rocker, m.</param>
/// <param name="RockerLength">Rocker (the clamp arm), m.</param>
/// <param name="CrankMass">Crank mass, kg.</param>
/// <param name="CouplerMass">Coupler mass, kg.</param>
/// <param name="RockerMass">Rocker mass, kg.</param>
/// <param name="OverCenter">How far past the toggle the crank rests when latched, rad.</param>
/// <param name="OpenStop">
/// How far short of the toggle the crank's open stop sits, rad. The
/// default, 30°, is close enough that the band is still stretched there
/// (26 N of the 50 N latched), so it holds the released clamp open against
/// the stop: the clamp has two rest poses, shut and open.
/// </param>
/// <param name="StopStiffness">Torsional stiffness of both crank stops, N·m/rad.</param>
/// <param name="StopDamping">Torsional damping of both crank stops, N·m·s/rad.</param>
/// <param name="BandLength">Length of the load band in the latched pose, m.</param>
/// <param name="BandStrain">The band's stretch in the latched pose, as a fraction of its rest length.</param>
/// <param name="BandDamping">The band's viscous damping c, N·s/m.</param>
public sealed record FourBarClampGeometry(
    double GroundLength = 0.20,
    double CrankLength = 0.05,
    double CouplerLength = 0.17,
    double RockerLength = 0.10,
    double CrankMass = 0.05,
    double CouplerMass = 0.08,
    double RockerMass = 0.10,
    double OverCenter = 3 * Math.PI / 180,
    double OpenStop = Math.PI / 6,
    double StopStiffness = 50.0,
    double StopDamping = 0.1,
    double BandLength = 0.10,
    double BandStrain = 0.25,
    double BandDamping = 0.5);

/// <summary>
/// The four-bar toggle clamp (PLAN.md M8): an over-center lock that its
/// load holds shut.
/// <code>
///                       B ○──────────● band anchor (body)
///              coupler ╱  │
///                    ╱    │ rocker (clamp arm)
///           crank  ○      │
///                ╱ A      │
///   crank pivot ●━━━━━━━━━● rocker pivot      (ground)
/// </code>
/// A crank turns on a ground pivot at the origin, a rocker on a second
/// ground pivot at +x, and a coupler joins the crank's end A to the
/// rocker's end B. The <b>toggle</b> is where the crank and coupler line up:
/// B is then as far from the crank pivot as it can get (crank + coupler),
/// so a load pulling B toward the crank pivot runs straight down the crank
/// and coupler into the pivot and puts no torque on the crank.
///
/// The load is a stretched elastic band from B to an anchor on the body,
/// pulling B toward the crank pivot (−x) with
/// <see cref="Create"/>'s clamp tension in the latched pose: the clamp is
/// holding something elastic shut, like a workpiece, a loaded sheet or a
/// ligament. On either side of the toggle that pull turns the crank further
/// from it. Latched <see cref="FourBarClampGeometry.OverCenter"/> past the
/// toggle, the band drives the crank into its stop instead of opening the
/// clamp, so it holds with no torque, harder the harder it is loaded.
/// Pulling the crank back over the toggle releases it: the band throws the
/// clamp open until the crank's open stop catches it, and still stretched,
/// holds it open there.
///
/// The stop is placed so the latched pose is an exact equilibrium: a
/// penalty stop pushes only once the crank is past it, so it sits its
/// torque / k short of the latched angle (as the wing's preloaded stops
/// do). There is no gravity by default, as for a clamp working in a
/// horizontal plane, so the band alone sets what holds.
///
/// Pin ends A are the moving links' ends: the crank pivot's joint angle is
/// the crank's world angle, and a positive torque there turns the crank
/// counterclockwise, toward and past the toggle.
/// </summary>
public static class FourBarClamp
{
    public const string Crank = "crank";
    public const string Coupler = "coupler";
    public const string Rocker = "rocker";
    public const string CrankPivot = "crank_pivot";
    public const string Knuckle = "knuckle";
    public const string ClampPin = "clamp_pin";
    public const string RockerPivot = "rocker_pivot";
    public const string Band = "band";
    public const string ReleaseTwitch = "release_twitch";

    /// <summary>The rocker's world angle where the crank and coupler line up (the toggle), rad.</summary>
    public static double ToggleRockerAngle(FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        var reach = g.CrankLength + g.CouplerLength;
        var cos = ((reach * reach) - (g.GroundLength * g.GroundLength) - (g.RockerLength * g.RockerLength)) / (2 * g.GroundLength * g.RockerLength);
        if (!(Math.Abs(cos) <= 1))
        {
            throw new ArgumentException("The crank and coupler cannot line up with the rocker: no toggle.", nameof(geometry));
        }

        return Math.Acos(cos);
    }

    /// <summary>The crank's world angle at the toggle, rad: it points straight at the rocker's end B.</summary>
    public static double ToggleCrankAngle(FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        var psi = ToggleRockerAngle(g);
        return Math.Atan2(g.RockerLength * Math.Sin(psi), g.GroundLength + (g.RockerLength * Math.Cos(psi)));
    }

    /// <summary>The crank's world angle when latched: <see cref="FourBarClampGeometry.OverCenter"/> past the toggle, rad.</summary>
    public static double LatchedCrankAngle(FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        return ToggleCrankAngle(g) + g.OverCenter;
    }

    /// <summary>
    /// The rocker's world angle for a crank at world angle
    /// <paramref name="crankAngle"/>, on the branch with B above the ground
    /// line, rad.
    /// </summary>
    public static double RockerAngle(double crankAngle, FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        var a = new Vector2d(g.CrankLength * Math.Cos(crankAngle), g.CrankLength * Math.Sin(crankAngle));
        var pivot = new Vector2d(g.GroundLength, 0);

        // B is where the coupler's circle about A meets the rocker's circle about its pivot.
        var span = pivot - a;
        var d = span.Length;
        if (!(d <= g.CouplerLength + g.RockerLength && d >= Math.Abs(g.CouplerLength - g.RockerLength)))
        {
            throw new ArgumentException($"The linkage cannot close with the crank at {crankAngle} rad.", nameof(crankAngle));
        }

        var along = ((g.CouplerLength * g.CouplerLength) - (g.RockerLength * g.RockerLength) + (d * d)) / (2 * d);
        var across = Math.Sqrt(Math.Max(0, (g.CouplerLength * g.CouplerLength) - (along * along)));
        var unit = span / d;
        var normal = new Vector2d(-unit.Y, unit.X);
        var b1 = a + (along * unit) + (across * normal);
        var b2 = a + (along * unit) - (across * normal);
        var b = b1.Y >= b2.Y ? b1 : b2;
        return Math.Atan2(b.Y, b.X - g.GroundLength);
    }

    /// <summary>The rocker's end B with the crank at world angle <paramref name="crankAngle"/>, m.</summary>
    public static Vector2d EndB(double crankAngle, FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        var psi = RockerAngle(crankAngle, g);
        return new Vector2d(g.GroundLength + (g.RockerLength * Math.Cos(psi)), g.RockerLength * Math.Sin(psi));
    }

    /// <summary>The band's anchor on the body: <see cref="FourBarClampGeometry.BandLength"/> toward −x from B in the latched pose, m.</summary>
    public static Vector2d BandAnchor(FourBarClampGeometry? geometry = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        return EndB(LatchedCrankAngle(g), g) - new Vector2d(g.BandLength, 0);
    }

    /// <summary>
    /// A twitch on the crank: a triangular torque pulse of angular impulse
    /// <paramref name="impulse"/> N·m·s from <paramref name="start"/> for
    /// <paramref name="duration"/>. A negative impulse pulls the crank back
    /// toward the toggle, which releases a latched clamp.
    /// </summary>
    public static JointActuator Twitch(double impulse, double start, double duration)
    {
        if (!double.IsFinite(impulse) || !(start >= 0) || !(duration > 0))
        {
            throw new ArgumentException("A twitch needs a finite impulse, a start time of at least 0 and a positive duration.");
        }

        return new JointActuator(
            ReleaseTwitch,
            CrankPivot,
            new Schedule([new SchedulePoint(start, 0), new SchedulePoint(start + (duration / 2), 2 * impulse / duration), new SchedulePoint(start + duration, 0)]));
    }

    /// <summary>
    /// The clamp at rest holding its band at <paramref name="clampTension"/>
    /// N in the latched pose. The crank starts at
    /// <paramref name="crankAngle"/> (default: latched, an exact
    /// equilibrium); the band and stops are always designed for the latched
    /// pose, so another start angle shows the same clamp elsewhere. Optional
    /// actuators act on it; gravity is off unless given.
    /// </summary>
    public static PlanarModel Create(
        double clampTension,
        FourBarClampGeometry? geometry = null,
        double? crankAngle = null,
        IEnumerable<JointActuator>? actuators = null,
        Vector2d? gravity = null)
    {
        var g = geometry ?? new FourBarClampGeometry();
        if (!(clampTension > 0) || !double.IsFinite(clampTension))
        {
            throw new ArgumentOutOfRangeException(nameof(clampTension), clampTension, "Must be finite and positive.");
        }

        if (!(g.OverCenter > 0 && g.OpenStop > 0 && g.StopStiffness > 0 && g.BandLength > 0 && g.BandStrain > 0))
        {
            throw new ArgumentException("The stops need a positive over-center angle, open angle and stiffness, and the band a positive length and strain.", nameof(geometry));
        }

        var toggle = ToggleCrankAngle(g);
        var latched = toggle + g.OverCenter;

        // The band: stretched by BandStrain in the latched pose, so EA = T / strain.
        var anchor = BandAnchor(g);
        var axialStiffness = clampTension / g.BandStrain;
        var restLength = g.BandLength / (1 + g.BandStrain);

        var crank = PlanarLink.SlenderBar(Crank, g.CrankMass, g.CrankLength);
        var coupler = PlanarLink.SlenderBar(Coupler, g.CouplerMass, g.CouplerLength);
        var rocker = PlanarLink.SlenderBar(Rocker, g.RockerMass, g.RockerLength);
        var band = new Cable(Band, Attachment.OnLink(Rocker, rocker.EndB), Attachment.Ground(anchor), restLength, axialStiffness, g.BandDamping);

        PinJoint[] Pins(JointLimit? limit) =>
        [
            new(CrankPivot, Attachment.OnLink(Crank, crank.EndA), Attachment.Ground(Vector2d.Zero), limit: limit),
            new(Knuckle, Attachment.OnLink(Coupler, coupler.EndA), Attachment.OnLink(Crank, crank.EndB)),
            new(ClampPin, Attachment.OnLink(Coupler, coupler.EndB), Attachment.OnLink(Rocker, rocker.EndB)),
            new(RockerPivot, Attachment.OnLink(Rocker, rocker.EndA), Attachment.Ground(new Vector2d(g.GroundLength, 0))),
        ];

        void Pose(PlanarSystem system, double[] state, double theta)
        {
            var psi = RockerAngle(theta, g);
            var a = new Vector2d(g.CrankLength * Math.Cos(theta), g.CrankLength * Math.Sin(theta));
            var b = new Vector2d(g.GroundLength + (g.RockerLength * Math.Cos(psi)), g.RockerLength * Math.Sin(psi));
            system.SetBody(state, 0, new Pose2d(0.5 * a, theta), default);
            system.SetBody(state, 1, new Pose2d(0.5 * (a + b), Math.Atan2(b.Y - a.Y, b.X - a.X)), default);
            system.SetBody(state, 2, new Pose2d(0.5 * (new Vector2d(g.GroundLength, 0) + b), psi), default);
        }

        var grav = gravity ?? Vector2d.Zero;

        // The torque the latched crank needs, found on the clamp without stops,
        // is what the closing stop must carry there.
        var free = new PlanarSystem([crank, coupler, rocker], grav, Pins(null), [band]);
        var freeState = free.CreateState().Values;
        Pose(free, freeState, latched);
        var needed = free.SolveHoldingTorques(0, freeState, [CrankPivot])[CrankPivot];
        if (!(needed < 0))
        {
            throw new ArgumentException("The band does not drive the latched crank into its stop: it is not past center.", nameof(geometry));
        }

        // The closing stop pushes back with k·(θ − upper), so it sits |needed|/k short of the latched angle.
        var limit = new JointLimit(toggle - g.OpenStop, latched + (needed / g.StopStiffness), g.StopStiffness, g.StopDamping);
        var system = new PlanarSystem([crank, coupler, rocker], grav, Pins(limit), [band], actuators: actuators);
        var state = system.CreateState().Values;
        Pose(system, state, crankAngle ?? latched);
        return new PlanarModel("four_bar_clamp", system, state);
    }
}
