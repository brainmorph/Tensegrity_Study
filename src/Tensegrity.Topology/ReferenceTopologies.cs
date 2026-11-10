using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Topology;

/// <summary>
/// The study's reference topologies with their run settings. These are
/// exported as the JSON files in <c>topologies/</c>. Each one sets up the
/// same system and initial state as the regression scenario of the same
/// name (a test holds them bit-identical), so a CLI run of a reference
/// file reproduces what the baselines protect.
/// </summary>
public static class ReferenceTopologies
{
    /// <summary>Working step for every reference topology, s. See the dt study in topologies/README.md.</summary>
    public const double Dt = 1e-4;

    /// <summary>Default run length, s.</summary>
    public const double Duration = 2.0;

    /// <summary>Steps per recorded frame: 100 frames per second at <see cref="Dt"/>.</summary>
    public const int FrameStride = 100;

    // Planar X parameters (PLAN.md M2).
    public const double XWidth = 0.6;
    public const double XHeight = 1.0;
    public const double XStrutMass = 0.5;
    public const double XAxialStiffness = 5000.0;
    public const double XCableDamping = 3.0;
    public const double XStrutCompression = 50.0;
    public const double XSideCableTension = 20.0;
    public const double XKickRate = 0.3;

    // Locking arm parameters (PLAN.md M5): the LockingArmGeometry defaults, plus these.
    public const double ArmKickRate = -2.0;
    public const double ArmCollapseScale = 1.10;

    // Locking wing parameters (PLAN.md M6): the LockingWingGeometry defaults, plus these.
    public const double WingGustImpulse = 0.2;
    // Off the step grid, so the gust's breakpoints split steps at every dt.
    public const double WingGustStart = 0.10037;
    public const double WingGustDuration = 0.02;
    public const double WingCollapseScale = 1.08;

    // Preloaded wing: hyperextension stops at the elbow and wrist, each
    // preloaded by about its own joint's gravity moment (0.486 and 0.074 N m).
    public const double WingElbowPreload = 0.5;
    public const double WingWristPreload = 0.075;

    /// <summary>The wing with hyperextension stops, its elbow and wrist tendons preloading them.</summary>
    public static LockingWingGeometry PreloadedWing { get; } =
        new(HyperextensionStops: true, ElbowPreload: WingElbowPreload, WristPreload: WingWristPreload);

    public static IReadOnlyList<string> Names { get; } =
        ["pendulum", "cable_oscillator", "planar_x", "planar_x_rest", "planar_x_free", "locking_arm", "locking_arm_collapse", "ground_drop", "perturbed_bar", "locking_wing", "locking_wing_collapse"];

    public static PlanarModel Create(string name) => name switch
    {
        "pendulum" => Pendulum(),
        "cable_oscillator" => CableOscillator(),
        "planar_x" => PlanarX(),
        "planar_x_rest" => PlanarXRest(),
        "planar_x_free" => PlanarXFree(),
        "locking_arm" => LockingArmKicked(),
        "locking_arm_collapse" => LockingArmCollapse(),
        "ground_drop" => GroundDrop(),
        "perturbed_bar" => PerturbedBarCase(),
        "locking_wing" => LockingWingGust(),
        "locking_wing_collapse" => LockingWingCollapse(),
        _ => throw new ArgumentException($"Unknown reference topology '{name}'. Known: {string.Join(", ", Names)}.", nameof(name)),
    };

    private static SimulationDocument DefaultSimulation() => new() { Dt = Dt, Duration = Duration, FrameStride = FrameStride };

    /// <summary>A 1 kg, 1 m slender bar pinned at one end to the origin, released from rest 1 rad from hanging.</summary>
    public static PlanarModel Pendulum()
    {
        var link = PlanarLink.SlenderBar("bar", mass: 1.0, length: 1.0);
        var pivot = new PinJoint("pivot", Attachment.OnLink(link.Id, link.EndA), Attachment.Ground(Vector2d.Zero));
        var system = new PlanarSystem([link], PlanarSystem.StandardGravity, [pivot]);
        var toCenter = -link.EndA;
        var hanging = Math.Atan2(system.Gravity.Y, system.Gravity.X) - Math.Atan2(toCenter.Y, toCenter.X);
        var theta = hanging + 1.0;
        var state = system.CreateState().Values;
        system.SetBody(state, 0, new Pose2d(Vector2d.Zero + toCenter.Rotated(theta), theta), new Twist2d(Vector2d.Cross(0.0, toCenter.Rotated(theta)), 0.0));
        return new PlanarModel(
            "pendulum",
            system,
            state,
            "Slender bar (1 kg, 1 m) pinned at one end to the origin, released from rest 1 rad from hanging, standard gravity.",
            DefaultSimulation());
    }

    /// <summary>
    /// A 2 kg link hanging from a ground anchor by one damped cable at its
    /// center of mass, released 3 static stretches below equilibrium so it
    /// bounces through taut, recoiling and slack.
    /// </summary>
    public static PlanarModel CableOscillator()
    {
        const double mass = 2.0;
        const double restLength = 1.0;
        const double axialStiffness = 2000.0;
        const double stiffness = axialStiffness / restLength;
        var link = PlanarLink.SlenderBar("weight", mass, 0.2);
        var cable = new Cable("tendon", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("weight", Vector2d.Zero), restLength, axialStiffness, 8.0);
        var system = new PlanarSystem([link], PlanarSystem.StandardGravity, cables: [cable]);
        var staticStretch = mass * -PlanarSystem.StandardGravity.Y / stiffness;
        var equilibriumY = -restLength - staticStretch;
        var state = system.CreateState().Values;
        system.SetBody(state, 0, new Pose2d(new Vector2d(0, equilibriumY - (3.0 * staticStretch)), 0), new Twist2d(new Vector2d(0, 0.0), 0));
        return new PlanarModel(
            "cable_oscillator",
            system,
            state,
            "2 kg link hanging from a ground anchor by one cable at its center of mass (L0 1 m, EA 2000 N, c 8 N s/m), released at rest 3 static stretches below equilibrium; bounces through taut, recoiling and slack.",
            DefaultSimulation());
    }

    /// <summary>The X on pinned feet under gravity, at rest in its prestressed equilibrium.</summary>
    public static PlanarModel PlanarXRest()
    {
        var model = PlanarTensegrityX.PinnedFeet(XWidth, XHeight, XStrutMass, XAxialStiffness, XCableDamping, XSideCableTension, PlanarSystem.StandardGravity);
        return model.With(
            name: "planar_x_rest",
            description: "Planar two-strut tensegrity, 0.6 x 1.0 m, 0.5 kg struts, feet pinned to the ground, top/left/right cables (EA 5000 N, c 3 N s/m, side prestress 20 N), standard gravity, at rest: it holds its shape with no actuators.",
            simulation: DefaultSimulation());
    }

    /// <summary>The X on pinned feet under gravity, with strut s1 kicked about its foot.</summary>
    public static PlanarModel PlanarX()
    {
        var model = PlanarXRest();
        var system = model.System;
        var state = model.CreateInitialState().Values;
        var pose = system.GetPose(state, 0);
        var r = pose.Position - new Vector2d(-XWidth / 2, 0);
        system.SetBody(state, 0, pose, new Twist2d(Vector2d.Cross(XKickRate, r), XKickRate));
        return model.With(
            state,
            "planar_x",
            "Planar two-strut tensegrity, 0.6 x 1.0 m, 0.5 kg struts, feet pinned to the ground, top/left/right cables (EA 5000 N, c 3 N s/m, side prestress 20 N), standard gravity. Strut s1 kicked at 0.3 rad/s about its foot; it rings down back to its shape.");
    }

    private const string ArmSummary =
        "Two-link arm, upper 0.30 m / 0.20 kg pinned to the body at the shoulder, lower 0.35 m / 0.12 kg pinned at the elbow, standard gravity, no muscle torque. Each joint is held straight out by one tendon (EA 1000 N, c 150 N s/m) from a point 3 cm behind and 4-5 cm above it";

    /// <summary>The locking arm held straight out, with the lower link kicked downward about the elbow.</summary>
    public static PlanarModel LockingArmKicked()
    {
        var model = LockingArm.Create();
        var system = model.System;
        var state = model.CreateInitialState().Values;
        var pose = system.GetPose(state, 1);
        var elbow = system.GetPose(state, 0).ToWorld(system.Links[0].EndB);
        system.SetBody(state, 1, pose, new Twist2d(Vector2d.Cross(ArmKickRate, pose.Position - elbow), ArmKickRate));
        return model.With(
            state,
            "locking_arm",
            ArmSummary + ". The lower link is kicked down at 2 rad/s about the elbow; the lock holds and it rings back to straight out.",
            DefaultSimulation());
    }

    /// <summary>The locking arm with its elbow tendon lengthened past the lock threshold, released from straight out.</summary>
    public static PlanarModel LockingArmCollapse()
    {
        var model = LockingArm.Create(elbowRestLengthScale: ArmCollapseScale);
        return model.With(
            name: "locking_arm_collapse",
            description: ArmSummary + ", except the elbow tendon is 10% longer than the lock needs, past its threshold of about 8.2%. Released at rest from straight out, the lower link collapses, strikes the elbow's fold stop at 150 degrees (k 20 N m/rad, c 0.2 N m s/rad) and rebounds.",
            simulation: DefaultSimulation());
    }

    /// <summary>A bar dropped tilted on to the ground line: it lands, bounces, slides and comes to rest.</summary>
    public static PlanarModel GroundDrop() =>
        DroppedBar.Create().With(
            name: "ground_drop",
            description: "Slender bar, 1 m / 1 kg, with a contact point at each end, dropped on to the ground line y = 0 under standard gravity: center 0.4 m up, tilted 30 degrees, moving at 1.5 m/s in +x and turning at -2 rad/s. Ground: normal k 1e4 N/m, c 60 N s/m; Coulomb friction mu 0.5 through a tangential spring 1e4 N/m and damper 60 N s/m in series with a slider. It lands on one end, bounces, slides and comes to rest lying flat.",
            simulation: DefaultSimulation());

    /// <summary>The bar at rest on the ground, which falls away under it; then a knock at one end.</summary>
    public static PlanarModel PerturbedBarCase() =>
        PerturbedBar.Create().With(
            name: "perturbed_bar",
            description: "The ground_drop bar (1 m, 1 kg, same ground) lying at rest on the ground line. At 0.1 s the ground falls 0.1 m in 10 ms, faster than the bar can follow; the bar falls after it and lands. At 0.5 s a 5 ms triangular pulse of (0.3, 0.6) N s at the right end knocks it up and sideways; it rocks, lands and settles flat by about 0.8 s.",
            simulation: DefaultSimulation());

    private const string WingSummary =
        "Three-link wing: upper 0.30 m / 0.20 kg pinned to the body at the shoulder, lower 0.35 m / 0.12 kg at the elbow, hand 0.25 m / 0.06 kg at the wrist, standard gravity, no muscle torque. Each joint is held straight out by one tendon (EA 1000 N, c 150 N s/m) from a point behind and above it; the elbow and wrist have fold stops at 150 degrees";

    /// <summary>The locking wing held straight out, hit by a downward gust at the wingtip.</summary>
    public static PlanarModel LockingWingGust() =>
        LockingWing.Create(loads: [LockingWing.WingtipGust(WingGustImpulse, WingGustStart, WingGustDuration)]).With(
            name: "locking_wing",
            description: WingSummary + ". At 0.1 s a downward gust of 0.2 N s over 20 ms (peak 20 N) hits the wingtip. The wrist bends to about 23 degrees, two thirds of the way to its fold; on the rebound the hand flicks up to about 52 degrees above straight with its tendon slack (the joints have no hyperextension stops). The lock holds and the wing rings back to straight out.",
            simulation: DefaultSimulation());

    /// <summary>The locking wing with its wrist tendon lengthened past the lock threshold, released from straight out.</summary>
    public static PlanarModel LockingWingCollapse() =>
        LockingWing.Create(restLengthScales: [1, 1, WingCollapseScale]).With(
            name: "locking_wing_collapse",
            description: WingSummary + ", except the wrist tendon is 8% longer than the lock needs, past its threshold of about 6.3%. Released at rest from straight out, the hand collapses and swings, hanging from the wrist.",
            simulation: DefaultSimulation());

    /// <summary>The free-floating X with no gravity, after opposite twists on its struts (zero total momentum).</summary>
    public static PlanarModel PlanarXFree()
    {
        var model = PlanarTensegrityX.FreeFloating(XWidth, XHeight, XStrutMass, XAxialStiffness, XCableDamping, XStrutCompression);
        var system = model.System;
        var state = model.CreateInitialState().Values;
        var twist = new Twist2d(new Vector2d(0.1, 0.05), 0.3);
        system.SetBody(state, 0, system.GetPose(state, 0), twist);
        system.SetBody(state, 1, system.GetPose(state, 1), new Twist2d(-twist.Linear, -twist.Angular));
        return model.With(
            state,
            "planar_x_free",
            "Planar two-strut tensegrity, 0.6 x 1.0 m, 0.5 kg struts, free-floating, four cables (EA 5000 N, c 3 N s/m, strut compression 50 N), no gravity. Opposite twists on the two struts, zero total momentum; it rings down back to its shape.",
            DefaultSimulation());
    }
}
