using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// The planar two-strut tensegrity (PLAN.md M2, test 9) with the study's
/// reference parameters, and the perturbations used to show it is stable.
/// </summary>
public static class PlanarX
{
    public const double Width = 0.6;
    public const double Height = 1.0;
    public const double StrutMass = 0.5;
    public const double AxialStiffness = 5000.0;
    public const double CableDamping = 3.0;

    /// <summary>Strut compression of the free-floating X, N.</summary>
    public const double StrutCompression = 50.0;

    /// <summary>Side-cable tension of the pinned X, N.</summary>
    public const double SideCableTension = 20.0;

    /// <summary>Angular rate of the kick given to strut s1 about its pinned foot, rad/s.</summary>
    public const double KickRate = 0.3;

    public static double Diagonal => Math.Sqrt((Width * Width) + (Height * Height));

    public static PlanarModel Free() =>
        PlanarTensegrityX.FreeFloating(Width, Height, StrutMass, AxialStiffness, CableDamping, StrutCompression);

    public static PlanarModel Pinned() =>
        PlanarTensegrityX.PinnedFeet(Width, Height, StrutMass, AxialStiffness, CableDamping, SideCableTension, PlanarSystem.StandardGravity);

    /// <summary>
    /// Opposite twists on the two free struts, which share a center: zero
    /// total linear and angular momentum, so the X cannot drift or spin as a
    /// whole and any lasting change would be a change of shape.
    /// </summary>
    public static SystemState PerturbedFree(PlanarModel model)
    {
        var state = model.CreateInitialState();
        var system = model.System;
        var twist = new Twist2d(new Vector2d(0.1, 0.05), 0.3);
        system.SetBody(state.Values, 0, system.GetPose(state.Values, 0), twist);
        system.SetBody(state.Values, 1, system.GetPose(state.Values, 1), new Twist2d(-twist.Linear, -twist.Angular));
        return state;
    }

    /// <summary>Strut s1 turning about its pinned foot at <see cref="KickRate"/>, consistent with the pin.</summary>
    public static SystemState KickedPinned(PlanarModel model)
    {
        var state = model.CreateInitialState();
        var system = model.System;
        var pose = system.GetPose(state.Values, 0);
        var r = pose.Position - new Vector2d(-Width / 2, 0);
        system.SetBody(state.Values, 0, pose, new Twist2d(Vector2d.Cross(KickRate, r), KickRate));
        return state;
    }
}

/// <summary>
/// Test 9 (PLAN.md section 7) reference run: the X standing under gravity on
/// two pinned feet, held by three prestressed cables, after a kick to one
/// strut. Exercises pins, cables, gravity and damping together.
/// </summary>
public sealed class PlanarXScenario : IReferenceScenario
{
    public string Name => "planar_x";

    public string Description =>
        "Planar two-strut tensegrity, 0.6 x 1.0 m, 0.5 kg struts, feet pinned to the ground, top/left/right cables (EA 5000 N, c 3 N s/m, side prestress 20 N), standard gravity. Strut s1 kicked at 0.3 rad/s about its foot. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = PlanarX.Pinned();
        return PlanarRecording.Record(model, PlanarX.KickedPinned(model));
    }
}

/// <summary>
/// Test 9 (PLAN.md section 7) reference run: the free-floating X with no
/// gravity, in prestressed self-equilibrium, after a zero-momentum
/// perturbation of its shape.
/// </summary>
public sealed class PlanarXFreeScenario : IReferenceScenario
{
    public string Name => "planar_x_free";

    public string Description =>
        "Planar two-strut tensegrity, 0.6 x 1.0 m, 0.5 kg struts, free-floating, four cables (EA 5000 N, c 3 N s/m, strut compression 50 N), no gravity. Opposite twists ((0.1, 0.05) m/s, 0.3 rad/s) on the two struts, zero total momentum. RK4 with event splitting, dt 1e-4 s, 1 s, sampled every 0.05 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var model = PlanarX.Free();
        return PlanarRecording.Record(model, PlanarX.PerturbedFree(model));
    }
}
