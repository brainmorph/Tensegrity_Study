using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>Records poses, tensions, pin reactions and energy for a planar structure run: 1 s at dt 1e-4 s, sampled every 0.05 s.</summary>
internal static class PlanarRecording
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
                system.GetPinReactions(s.Time, s.Values, reactions);
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
