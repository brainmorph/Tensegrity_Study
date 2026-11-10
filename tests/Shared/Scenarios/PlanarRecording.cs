using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Topology;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Records poses, tensions, pin reactions, ground contact forces and energy
/// for a planar structure run: 1 s at dt 1e-4 s, sampled every 0.05 s.
/// </summary>
internal static class PlanarRecording
{
    public const double Dt = 1e-4;
    public const long Steps = 10_000;
    public const long Stride = 500;

    /// <summary>
    /// Records the run. For each pin in <paramref name="limitPins"/> it also
    /// records the extremes over every step of its joint angle
    /// (<c>min_/max_joint_angle_&lt;pin&gt;</c>) and of its joint-limit torque
    /// (<c>max_limit_torque_&lt;pin&gt;</c>, largest magnitude), since a stop
    /// strike can fall between samples.
    /// </summary>
    public static IReadOnlyDictionary<string, double[]> Record(PlanarModel model, SystemState state, IReadOnlyList<string>? limitPins = null)
    {
        var system = model.System;
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var recorder = new SampleRecorder();
        var tensions = new double[system.Cables.Count];
        var reactions = new Vector2d[system.Pins.Count];
        var initialEnergy = system.TotalEnergy(state.Values);
        var maxBalanceError = 0.0;
        var maxPinResidual = 0.0;
        var contactForces = new Vector2d[system.ContactCount];
        var maxPenetration = double.NegativeInfinity;
        var watched = (limitPins ?? []).Select(id => system.Pins.Select(p => p.Id).ToList().IndexOf(id)).ToArray();
        if (watched.Contains(-1))
        {
            throw new ArgumentException($"Unknown pin in {string.Join(", ", limitPins!)}.", nameof(limitPins));
        }

        var limitTorques = new double[system.Pins.Count];
        var minAngle = watched.Select(_ => double.PositiveInfinity).ToArray();
        var maxAngle = watched.Select(_ => double.NegativeInfinity).ToArray();
        var maxTorque = new double[watched.Length];

        FixedStepRunner.Run(state, stepper, Dt, Steps, (step, s) =>
        {
            maxBalanceError = Math.Max(maxBalanceError, Math.Abs(system.TotalEnergy(s.Values) - initialEnergy));
            maxPinResidual = Math.Max(maxPinResidual, system.PinPositionResidual(s.Values));
            for (var i = 0; i < system.ContactCount; i++)
            {
                maxPenetration = Math.Max(maxPenetration, system.GroundLine!.Penetration(system.GetContactPoint(s.Values, i).Y));
            }

            if (watched.Length > 0)
            {
                system.GetLimitTorques(s.Values, limitTorques);
                for (var w = 0; w < watched.Length; w++)
                {
                    var angle = system.GetJointAngle(s.Values, watched[w]).Angle;
                    minAngle[w] = Math.Min(minAngle[w], angle);
                    maxAngle[w] = Math.Max(maxAngle[w], angle);
                    maxTorque[w] = Math.Max(maxTorque[w], Math.Abs(limitTorques[watched[w]]));
                }
            }

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

            if (contactForces.Length > 0)
            {
                system.GetContactForces(s.Values, contactForces);
                for (var i = 0; i < contactForces.Length; i++)
                {
                    var id = system.GroundLine!.Points[i].Id;
                    recorder.Add($"contact_{id}_normal", contactForces[i].Y);
                    recorder.Add($"contact_{id}_friction", contactForces[i].X);
                }
            }

            recorder.Add("kinetic_energy", system.KineticEnergy(s.Values));
            recorder.Add("potential_energy", system.GravitationalPotentialEnergy(s.Values));
            recorder.Add("elastic_energy", system.ElasticEnergy(s.Values));
            recorder.Add("dissipated_energy", system.DissipatedEnergy(s.Values));
        });

        recorder.Add("max_energy_balance_error", maxBalanceError);
        if (reactions.Length > 0)
        {
            recorder.Add("max_pin_residual", maxPinResidual);
        }

        if (contactForces.Length > 0)
        {
            recorder.Add("max_ground_penetration", maxPenetration);
        }

        for (var w = 0; w < watched.Length; w++)
        {
            var id = system.Pins[watched[w]].Id;
            recorder.Add($"min_joint_angle_{id}", minAngle[w]);
            recorder.Add($"max_joint_angle_{id}", maxAngle[w]);
            recorder.Add($"max_limit_torque_{id}", maxTorque[w]);
        }

        recorder.Add("event_count", stepper.EventCount);
        recorder.Add("unresolved_steps", stepper.UnresolvedSteps);
        return recorder.ToOutput();
    }
}
