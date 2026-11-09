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

/// <summary>Records poses, tensions, pin reactions and energy for a planar X run.</summary>
internal static class PlanarXRecording
{
    public const double Dt = 1e-4;
    public const long Steps = 10_000;
    public const long Stride = 500;

    public static IReadOnlyDictionary<string, double[]> Record(PlanarModel model, SystemState state)
    {
        var system = model.System;
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var recorder = new SampleRecorder();
        var tensions = new double[system.Cables.Count];
        var reactions = new Vector2d[system.Pins.Count];
        var initialEnergy = system.TotalEnergy(state.Values);
        var maxBalanceError = 0.0;
        var maxPinResidual = 0.0;

        FixedStepRunner.Run(state, stepper, Dt, Steps, (step, s) =>
        {
            maxBalanceError = Math.Max(maxBalanceError, Math.Abs(system.TotalEnergy(s.Values) - initialEnergy));
            maxPinResidual = Math.Max(maxPinResidual, system.PinPositionResidual(s.Values));
            if (step % Stride != 0)
            {
                return;
            }

            recorder.Add("time", s.Time);
            for (var b = 0; b < system.BodyCount; b++)
            {
                var id = system.Links[b].Id;
                var pose = system.GetPose(s.Values, b);
                recorder.Add($"{id}_x", pose.Position.X);
                recorder.Add($"{id}_y", pose.Position.Y);
                recorder.Add($"{id}_theta", pose.Angle);
            }

            system.GetCableTensions(s.Values, tensions);
            for (var c = 0; c < tensions.Length; c++)
            {
                recorder.Add($"tension_{system.Cables[c].Id}", tensions[c]);
            }

            if (reactions.Length > 0)
            {
                system.GetPinReactions(s.Values, reactions);
                for (var p = 0; p < reactions.Length; p++)
                {
                    recorder.Add($"reaction_{system.Pins[p].Id}_x", reactions[p].X);
                    recorder.Add($"reaction_{system.Pins[p].Id}_y", reactions[p].Y);
                }
            }

            recorder.Add("kinetic_energy", system.KineticEnergy(s.Values));
            recorder.Add("potential_energy", system.GravitationalPotentialEnergy(s.Values));
            recorder.Add("elastic_energy", system.CableElasticEnergy(s.Values));
            recorder.Add("dissipated_energy", system.DissipatedEnergy(s.Values));
        });

        recorder.Add("max_energy_balance_error", maxBalanceError);
        if (reactions.Length > 0)
        {
            recorder.Add("max_pin_residual", maxPinResidual);
        }

        recorder.Add("event_count", stepper.EventCount);
        recorder.Add("unresolved_steps", stepper.UnresolvedSteps);
        return recorder.ToOutput();
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
        return PlanarXRecording.Record(model, PlanarX.KickedPinned(model));
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
        return PlanarXRecording.Record(model, PlanarX.PerturbedFree(model));
    }
}
