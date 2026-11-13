using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Analysis;

/// <summary>
/// Dimensions of a one-input coupled wing (brainstorm-coupled-wing.md), SI
/// units, angles in radians. The upper arm is bolted to the body: it is the
/// ground frame, from the shoulder at the origin to the elbow at
/// (<see cref="UpperLength"/>, 0). The forearm has two bones. The ulna
/// turns about the elbow, driven by the servo, and the hand turns about the
/// wrist at its far end. The radius runs beside the ulna, pinned to the
/// upper arm and to the hand at <see cref="RadiusOffset"/> from the elbow
/// and the wrist, in the direction <see cref="RadiusOffsetAngle"/>. Upper
/// arm, ulna, hand and radius form a four-bar, so the elbow angle sets the
/// wrist's: with the radius as long as the ulna it is a parallelogram, and
/// the hand stays parallel to the upper arm as the elbow folds (a Z-fold,
/// as in a folded bird wing).
/// <code>
///          anchor ●            R ○━━━━━━━━ radius ━━━━━━━○ H
///                  ╲ ligament                            ┃
///   upper arm ━━━━━━○━━━━━━━━━━ ulna ━━━━━━━━━━━━━━━━━━━━○━━━━ hand ━━━━
///   (ground)      elbow (servo)                       wrist
/// </code>
/// The elbow has two stops, open at <see cref="OpenAngle"/> and folded at
/// <see cref="FoldAngle"/>. An elastic ligament from an anchor on the upper
/// arm to a point on the ulna crosses the elbow: its line passes over the
/// joint at <see cref="OverCenterAngle"/>, so on either side of that it
/// presses the elbow toward the nearer stop. Its rest length is designed so
/// that the weaker of the two stops is pressed with <see cref="Preload"/>
/// beyond holding the wing's weight.
/// </summary>
public sealed record CoupledWingGeometry
{
    public double UpperLength { get; init; } = 0.30;

    public double UlnaLength { get; init; } = 0.35;

    public double UlnaMass { get; init; } = 0.12;

    /// <summary>Radius length, m; equal to the ulna's for a parallelogram.</summary>
    public double RadiusLength { get; init; } = 0.35;

    public double RadiusMass { get; init; } = 0.04;

    public double HandLength { get; init; } = 0.25;

    public double HandMass { get; init; } = 0.06;

    /// <summary>How far the radius's pins sit from the elbow and the wrist, m.</summary>
    public double RadiusOffset { get; init; } = 0.02;

    /// <summary>
    /// Direction of those offsets: in the world for the pin on the upper
    /// arm, in the hand's frame for the pin on the hand. The four-bar jams
    /// where the ulna lines up with it, so it is chosen outside the elbow's
    /// travel (modulo 180°).
    /// </summary>
    public double RadiusOffsetAngle { get; init; } = 15 * Math.PI / 180;

    /// <summary>Elbow angle (the ulna's world angle) of the open pose, rad.</summary>
    public double OpenAngle { get; init; } = 0;

    /// <summary>Elbow angle of the folded pose, rad.</summary>
    public double FoldAngle { get; init; } = -150 * Math.PI / 180;

    /// <summary>Torsional stiffness of each elbow stop, N·m/rad.</summary>
    public double StopStiffness { get; init; } = 100;

    /// <summary>
    /// Torsional damping of each elbow stop, N·m·s/rad. The ligament throws
    /// the elbow at a stop with the whole barrier's energy, and the four-bar
    /// jams 15° beyond, so the stop must absorb the blow: near critical
    /// damping, or the wing bounces back over the barrier.
    /// </summary>
    public double StopDamping { get; init; } = 2.5;

    /// <summary>Elbow angle at which the ligament's line passes over the elbow, rad.</summary>
    public double OverCenterAngle { get; init; } = -75 * Math.PI / 180;

    /// <summary>Distance of the ligament's anchor on the upper arm from the elbow, m.</summary>
    public double AnchorRadius { get; init; } = 0.04;

    /// <summary>Distance of the ligament's insertion on the ulna from the elbow, along the ulna, m.</summary>
    public double InsertionRadius { get; init; } = 0.04;

    /// <summary>Ligament axial stiffness EA, N: elastic, since it must stretch over 25% to pass over center.</summary>
    public double LigamentStiffness { get; init; } = 200;

    /// <summary>Ligament viscous damping, N·s/m.</summary>
    public double LigamentDamping { get; init; } = 5;

    /// <summary>Torque pressing the weaker stop beyond holding the wing's weight, N·m.</summary>
    public double Preload { get; init; } = 0.5;

    public double Gravity { get; init; } = 9.81;

    /// <summary>The elbow, at the end of the upper arm.</summary>
    public Vector2d Elbow => new(UpperLength, 0);

    /// <summary>The radius's pin on the upper arm.</summary>
    public Vector2d RadiusRoot => Elbow + (RadiusOffset * new Vector2d(Math.Cos(RadiusOffsetAngle), Math.Sin(RadiusOffsetAngle)));

    /// <summary>The ligament's anchor on the upper arm: opposite the insertion when the elbow is at the over-center angle.</summary>
    public Vector2d LigamentAnchor => Elbow + (AnchorRadius * new Vector2d(Math.Cos(Math.PI + OverCenterAngle), Math.Sin(Math.PI + OverCenterAngle)));

    public void Validate()
    {
        foreach (var (name, value) in new[]
        {
            (nameof(UpperLength), UpperLength), (nameof(UlnaLength), UlnaLength), (nameof(UlnaMass), UlnaMass),
            (nameof(RadiusLength), RadiusLength), (nameof(RadiusMass), RadiusMass), (nameof(HandLength), HandLength),
            (nameof(HandMass), HandMass), (nameof(RadiusOffset), RadiusOffset), (nameof(StopStiffness), StopStiffness),
            (nameof(AnchorRadius), AnchorRadius), (nameof(InsertionRadius), InsertionRadius), (nameof(LigamentStiffness), LigamentStiffness),
            (nameof(Preload), Preload),
        })
        {
            if (!double.IsFinite(value) || value <= 0)
            {
                throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
            }
        }

        if (!(FoldAngle < OverCenterAngle && OverCenterAngle < OpenAngle))
        {
            throw new ArgumentException("The over-center angle must lie between the folded and open angles.");
        }

        if (!(InsertionRadius < UlnaLength))
        {
            throw new ArgumentException("The ligament must insert on the ulna.");
        }
    }
}

/// <summary>Where every bone of a <see cref="CoupledWingGeometry"/> is at one elbow angle.</summary>
/// <param name="Elbow">Elbow angle φ, the ulna's world angle, rad.</param>
/// <param name="Radius">The radius's world angle, from its root toward the hand, rad.</param>
/// <param name="Hand">The hand's world angle, from the wrist outward, rad.</param>
/// <param name="Wrist">The wrist's world position, m.</param>
/// <param name="RadiusTip">The radius's pin on the hand, m.</param>
public readonly record struct CoupledWingPose(double Elbow, double Radius, double Hand, Vector2d Wrist, Vector2d RadiusTip);

/// <summary>A rest pose of the coupled wing: a local minimum of its potential in the elbow angle.</summary>
/// <param name="Elbow">Elbow angle, rad.</param>
/// <param name="Energy">Potential energy there, J.</param>
/// <param name="Stiffness">d²V/dφ² there, N·m/rad.</param>
public readonly record struct CoupledWingRest(double Elbow, double Energy, double Stiffness);

/// <summary>
/// The statics of a <see cref="CoupledWingGeometry"/>, written independently
/// of the simulator. The wing has one degree of freedom, the elbow angle φ,
/// and the four-bar is solved in closed form (two circles meet at the
/// radius's pin on the hand). The potential is
/// <code>
/// V(φ) = g·Σ m·y_cm + ½·k·(L − L0)²  (ligament, when stretched) + ½·κ·δ²  (a stop, when pressed)
/// </code>
/// The servo holds φ with torque τ = dV/dφ, a central difference, and the
/// wing rests where τ = 0 and V is a minimum.
///
/// Constructing it also designs the wing: the ligament's rest length is
/// set by bisection so the weaker stop gets the geometry's preload, and
/// each stop is placed its own pressing torque / κ short of its design
/// pose, so both design poses are exact equilibria (as the preloaded
/// <c>LockingWing</c> places its stops).
/// </summary>
public sealed class CoupledWingStatics
{
    private const double Step = 1e-6;

    private readonly CoupledWingGeometry g;

    public CoupledWingStatics(CoupledWingGeometry geometry)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        geometry.Validate();
        g = geometry;

        // Rest length: shorter pulls harder at both stops. Find the one where
        // the weaker stop is pressed with exactly the preload.
        var low = 1e-6;
        var high = Math.Min(LigamentLength(g.OpenAngle), LigamentLength(g.FoldAngle));
        if (WeakerPreload(low) < g.Preload)
        {
            throw new ArgumentException("No ligament rest length gives the preload: the ligament cannot press both stops.");
        }

        for (var i = 0; i < 200; i++)
        {
            var mid = 0.5 * (low + high);
            if (WeakerPreload(mid) >= g.Preload)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }

        RestLength = low;
        OpenPreload = FreeTorque(g.OpenAngle, RestLength);
        FoldPreload = -FreeTorque(g.FoldAngle, RestLength);
        OpenStop = g.OpenAngle - (OpenPreload / g.StopStiffness);
        FoldStop = g.FoldAngle + (FoldPreload / g.StopStiffness);
    }

    public CoupledWingGeometry Geometry => g;

    /// <summary>The ligament's designed rest length, m.</summary>
    public double RestLength { get; }

    /// <summary>Torque pressing the elbow into the open stop in the open pose, N·m.</summary>
    public double OpenPreload { get; }

    /// <summary>Torque pressing the elbow into the folded stop in the folded pose, N·m.</summary>
    public double FoldPreload { get; }

    /// <summary>Angle of the open stop (the elbow's upper limit), rad: OpenPreload / κ short of the open pose.</summary>
    public double OpenStop { get; }

    /// <summary>Angle of the folded stop (the elbow's lower limit), rad.</summary>
    public double FoldStop { get; }

    /// <summary>The ligament's insertion on the ulna, in the ulna's frame measured from the elbow, m.</summary>
    public double InsertionAlongUlna => g.InsertionRadius;

    /// <summary>Solves the four-bar at elbow angle <paramref name="elbow"/>, on the branch that is the parallelogram for equal bones.</summary>
    public CoupledWingPose Pose(double elbow)
    {
        var wrist = g.Elbow + (g.UlnaLength * U(elbow));
        var root = g.RadiusRoot;

        // The radius's pin on the hand lies on a circle of radius RadiusOffset about the wrist and of RadiusLength about its root.
        var d = wrist - root;
        var dist = d.Length;
        var r1 = g.RadiusOffset;
        var r2 = g.RadiusLength;
        if (!(dist <= r1 + r2 && dist >= Math.Abs(r1 - r2)))
        {
            throw new ArgumentOutOfRangeException(nameof(elbow), elbow, "The four-bar cannot close at this elbow angle.");
        }

        var along = ((r1 * r1) - (r2 * r2) + (dist * dist)) / (2 * dist);
        var across = Math.Sqrt(Math.Max(0, (r1 * r1) - (along * along)));
        var unit = (root - wrist) / dist;
        var first = wrist + (along * unit) + (across * unit.Perpendicular);
        var second = wrist + (along * unit) - (across * unit.Perpendicular);

        // The parallelogram branch puts the hand pin where the root sits relative to the elbow: H − W = R − E.
        var parallel = wrist + (root - g.Elbow);
        var tip = (first - parallel).LengthSquared <= (second - parallel).LengthSquared ? first : second;
        var hand = Angle(tip - wrist) - g.RadiusOffsetAngle;
        return new CoupledWingPose(elbow, Angle(tip - root), hand, wrist, tip);
    }

    /// <summary>World pose (center of mass and angle) of the ulna, radius and hand, in that order.</summary>
    public Pose2d[] BodyPoses(double elbow)
    {
        var p = Pose(elbow);
        return
        [
            new Pose2d(g.Elbow + (g.UlnaLength / 2 * U(elbow)), elbow),
            new Pose2d(g.RadiusRoot + (g.RadiusLength / 2 * U(p.Radius)), p.Radius),
            new Pose2d(p.Wrist + (g.HandLength / 2 * U(p.Hand)), p.Hand),
        ];
    }

    /// <summary>Distance from the ligament's anchor to its insertion, m.</summary>
    public double LigamentLength(double elbow) => (g.LigamentAnchor - (g.Elbow + (g.InsertionRadius * U(elbow)))).Length;

    /// <summary>Ligament tension, N.</summary>
    public double LigamentTension(double elbow)
    {
        var stretch = LigamentLength(elbow) - RestLength;
        return stretch > 0 ? g.LigamentStiffness / RestLength * stretch : 0;
    }

    /// <summary>Gravitational potential of the three moving bones, J (from y = 0).</summary>
    public double GravityEnergy(double elbow)
    {
        var bodies = BodyPoses(elbow);
        return g.Gravity * ((g.UlnaMass * bodies[0].Position.Y) + (g.RadiusMass * bodies[1].Position.Y) + (g.HandMass * bodies[2].Position.Y));
    }

    /// <summary>Total potential V, J: gravity, ligament and pressed stops.</summary>
    public double Energy(double elbow) => GravityEnergy(elbow) + LigamentEnergy(elbow, RestLength) + StopEnergy(elbow);

    /// <summary>Torque the servo must apply to the ulna to hold elbow angle φ still, N·m: dV/dφ.</summary>
    public double ServoTorque(double elbow) => (Energy(elbow + Step) - Energy(elbow - Step)) / (2 * Step);

    /// <summary>
    /// Rest poses with the servo let go: local minima of V over a grid of
    /// <paramref name="samples"/> elbow angles spanning both stops (and 10°
    /// beyond), each refined by golden-section search.
    /// </summary>
    public IReadOnlyList<CoupledWingRest> RestPoses(int samples = 4001)
    {
        var from = FoldStop - (10 * Math.PI / 180);
        var to = OpenStop + (10 * Math.PI / 180);
        var angles = Enumerable.Range(0, samples).Select(i => from + ((to - from) * i / (samples - 1))).ToArray();
        var energies = angles.Select(Energy).ToArray();
        var rests = new List<CoupledWingRest>();
        for (var i = 1; i < samples - 1; i++)
        {
            if (energies[i] <= energies[i - 1] && energies[i] < energies[i + 1])
            {
                var at = GoldenSection(angles[i - 1], angles[i + 1]);
                var stiffness = (Energy(at + 1e-4) - (2 * Energy(at)) + Energy(at - 1e-4)) / 1e-8;
                rests.Add(new CoupledWingRest(at, Energy(at), stiffness));
            }
        }

        return rests;
    }

    private double LigamentEnergy(double elbow, double restLength)
    {
        var stretch = LigamentLength(elbow) - restLength;
        return stretch > 0 ? 0.5 * g.LigamentStiffness / restLength * stretch * stretch : 0;
    }

    private double StopEnergy(double elbow)
    {
        var past = elbow > OpenStop ? elbow - OpenStop : elbow < FoldStop ? FoldStop - elbow : 0;
        return 0.5 * g.StopStiffness * past * past;
    }

    /// <summary>The generalized torque from gravity and the ligament alone, −d(V_g + V_l)/dφ, N·m: positive opens the wing.</summary>
    private double FreeTorque(double elbow, double restLength)
    {
        double V(double phi) => GravityEnergy(phi) + LigamentEnergy(phi, restLength);
        return -(V(elbow + Step) - V(elbow - Step)) / (2 * Step);
    }

    private double WeakerPreload(double restLength) =>
        Math.Min(FreeTorque(g.OpenAngle, restLength), -FreeTorque(g.FoldAngle, restLength));

    private double GoldenSection(double a, double b)
    {
        var ratio = (Math.Sqrt(5) - 1) / 2;
        var c = b - (ratio * (b - a));
        var d = a + (ratio * (b - a));
        for (var i = 0; i < 200 && b - a > 1e-13; i++)
        {
            if (Energy(c) < Energy(d))
            {
                b = d;
            }
            else
            {
                a = c;
            }

            c = b - (ratio * (b - a));
            d = a + (ratio * (b - a));
        }

        return 0.5 * (a + b);
    }

    private static Vector2d U(double angle) => new(Math.Cos(angle), Math.Sin(angle));

    private static double Angle(Vector2d v) => Math.Atan2(v.Y, v.X);
}
