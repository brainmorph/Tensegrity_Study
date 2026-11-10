using System.Buffers.Binary;
using System.Text.Json;
using Tensegrity.Core.Bodies;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Output;

/// <summary>Step size, length and recording cadence of a run.</summary>
public sealed record RunSettings(double Dt, long Steps, int FrameStride, double EventTolerance = EventSplittingStepper.DefaultTimeTolerance)
{
    /// <summary>Settings for a run of <paramref name="duration"/> seconds; the duration must be a whole number of steps.</summary>
    public static RunSettings ForDuration(double dt, double duration, int frameStride, double eventTolerance = EventSplittingStepper.DefaultTimeTolerance)
    {
        if (!double.IsFinite(dt) || dt <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "Must be finite and positive.");
        }

        var steps = Math.Round(duration / dt);
        if (!(steps >= 1) || Math.Abs((steps * dt) - duration) > 1e-9 * Math.Max(1, duration))
        {
            throw new ArgumentException($"Duration {duration} s is not a whole number of {dt} s steps.", nameof(duration));
        }

        return new RunSettings(dt, (long)steps, frameStride, eventTolerance);
    }
}

/// <summary>Descriptive fields of a run manifest that do not affect the simulation.</summary>
public sealed record RunMetadata(
    string TopologyName,
    string? TopologyDescription,
    long Seed,
    string CreatedUtc,
    string SimVersion = "unknown",
    bool? SimDirty = null,
    string? TopologySource = null);

/// <summary>
/// Runs a system with event-split RK4 and writes a run directory
/// (PLAN.md 6): <c>frames.bin</c> first, then <c>manifest.json</c>, so a
/// directory with a manifest is a complete run. Frames are the float32
/// casts of the float64 state, so the same input gives byte-identical
/// frames (test 12). Diagnostics are checked at every step, not only at
/// recorded frames.
/// </summary>
public static class RunRecorder
{
    /// <summary>A strut is flagged when its midpoint compression falls below −this, N.</summary>
    public const double CompressionTolerance = 1e-6;

    public const string ConstraintMethod = "lagrange_multipliers_baumgarte";

    public static RunManifest Record(PlanarSystem system, SystemState state, RunSettings settings, RunMetadata metadata, string directory)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentNullException.ThrowIfNull(state);
        ArgumentNullException.ThrowIfNull(settings);
        ArgumentNullException.ThrowIfNull(metadata);
        ArgumentOutOfRangeException.ThrowIfLessThan(settings.FrameStride, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(settings.Steps, 1);
        if (Directory.Exists(directory) && Directory.EnumerateFileSystemEntries(directory).Any())
        {
            throw new IOException($"Run directory '{directory}' already exists and is not empty.");
        }

        Directory.CreateDirectory(directory);

        // Topology as it stands at t0, before the run changes the state.
        var topology = TopologyMapping.ToDocument(system, state.Values, metadata.TopologyName, metadata.TopologyDescription);
        var channels = Channels(system);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system, settings.EventTolerance);
        var diagnostics = new RunDiagnostics { CompressionTolerance = CompressionTolerance };
        var tracker = new Tracker(system);
        var frameBuffer = new byte[4 * channels.Count];
        var frameCount = 0;
        var initialEnergy = system.TotalEnergy(state.Values);

        using (var frames = new FileStream(Path.Combine(directory, RunFiles.Frames), FileMode.CreateNew, FileAccess.Write))
        {
            FixedStepRunner.Run(state, stepper, settings.Dt, settings.Steps, (step, s) =>
            {
                tracker.Observe(s.Time, s.Values, initialEnergy);
                if (step % settings.FrameStride != 0)
                {
                    return;
                }

                WriteFrame(system, s, frameBuffer);
                frames.Write(frameBuffer);
                frameCount++;

                var energy = diagnostics.Energy;
                energy.Time.Add(s.Time);
                energy.Kinetic.Add(system.KineticEnergy(s.Values));
                energy.Gravitational.Add(system.GravitationalPotentialEnergy(s.Values));
                energy.Elastic.Add(system.ElasticEnergy(s.Values));
                energy.Dissipated.Add(system.DissipatedEnergy(s.Values));
                energy.ActuatorWork.Add(system.ActuatorWork(s.Values));
                energy.ExternalWork.Add(system.ExternalWork(s.Values));
                energy.Total.Add(system.TotalEnergy(s.Values));
            });
        }

        tracker.Fill(diagnostics);
        diagnostics.EventCount = stepper.EventCount;
        diagnostics.UnresolvedEventSteps = stepper.UnresolvedSteps;

        var manifest = new RunManifest
        {
            CreatedUtc = metadata.CreatedUtc,
            SimVersion = metadata.SimVersion,
            SimDirty = metadata.SimDirty,
            Integrator = stepper.IntegratorName,
            ConstraintMethod = ConstraintMethod,
            ConstraintStabilization = system.Stabilization.Frequency,
            Dt = settings.Dt,
            Duration = settings.Steps * settings.Dt,
            TotalSteps = settings.Steps,
            FrameStride = settings.FrameStride,
            FrameCount = frameCount,
            EventTolerance = settings.EventTolerance,
            Gravity = [system.Gravity.X, system.Gravity.Y],
            Ground = system.GroundLine is null ? null : GroundModels.PenaltyCoulomb,
            GlobalDamping = system.GlobalDamping,
            Actuators = TopologyMapping.ActuatorDocuments(system),
            Seed = metadata.Seed,
            TopologySource = metadata.TopologySource,
            Topology = topology,
            FrameLayout = new FrameLayout { Channels = channels },
            Diagnostics = diagnostics,
        };

        File.WriteAllText(Path.Combine(directory, RunFiles.Manifest), JsonSerializer.Serialize(manifest, RunFiles.Json) + "\n");
        return manifest;
    }

    /// <summary>Frame channel names for a system, in the order <see cref="WriteFrame"/> writes them.</summary>
    public static List<string> Channels(PlanarSystem system)
    {
        var channels = new List<string> { FrameChannels.Time };
        foreach (var link in system.Links)
        {
            channels.Add(FrameChannels.LinkX(link.Id));
            channels.Add(FrameChannels.LinkY(link.Id));
            channels.Add(FrameChannels.LinkTheta(link.Id));
        }

        foreach (var cable in system.Cables)
        {
            channels.Add(FrameChannels.CableTension(cable.Id));
            channels.Add(FrameChannels.CableAx(cable.Id));
            channels.Add(FrameChannels.CableAy(cable.Id));
            channels.Add(FrameChannels.CableBx(cable.Id));
            channels.Add(FrameChannels.CableBy(cable.Id));
        }

        foreach (var pin in system.Pins)
        {
            channels.Add(FrameChannels.PinFx(pin.Id));
            channels.Add(FrameChannels.PinFy(pin.Id));
            channels.Add(FrameChannels.PinX(pin.Id));
            channels.Add(FrameChannels.PinY(pin.Id));
        }

        foreach (var actuator in system.Actuators)
        {
            channels.Add(FrameChannels.ActuatorTorque(actuator.Id));
        }

        foreach (var pin in system.Pins.Where(p => p.Limit is not null))
        {
            channels.Add(FrameChannels.PinLimitTorque(pin.Id));
        }

        foreach (var load in system.Loads)
        {
            channels.Add(FrameChannels.LoadFx(load.Id));
            channels.Add(FrameChannels.LoadFy(load.Id));
            channels.Add(FrameChannels.LoadX(load.Id));
            channels.Add(FrameChannels.LoadY(load.Id));
        }

        if (system.GroundLine is not null)
        {
            channels.Add(FrameChannels.GroundHeight);
        }

        foreach (var contact in system.GroundLine?.Points ?? [])
        {
            channels.Add(FrameChannels.ContactNormal(contact.Id));
            channels.Add(FrameChannels.ContactFriction(contact.Id));
            channels.Add(FrameChannels.ContactX(contact.Id));
            channels.Add(FrameChannels.ContactY(contact.Id));
        }

        return channels;
    }

    private static void WriteFrame(PlanarSystem system, SystemState state, byte[] buffer)
    {
        var y = state.Values;
        var i = 0;
        void Put(double value)
        {
            BinaryPrimitives.WriteSingleLittleEndian(buffer.AsSpan(4 * i), (float)value);
            i++;
        }

        Put(state.Time);
        for (var b = 0; b < system.BodyCount; b++)
        {
            var pose = system.GetPose(y, b);
            Put(pose.Position.X);
            Put(pose.Position.Y);
            Put(pose.Angle);
        }

        var tensions = new double[system.Cables.Count];
        system.GetCableTensions(y, tensions);
        for (var c = 0; c < tensions.Length; c++)
        {
            var cable = system.Cables[c];
            var a = system.WorldPoint(y, cable.A);
            var b = system.WorldPoint(y, cable.B);
            Put(tensions[c]);
            Put(a.X);
            Put(a.Y);
            Put(b.X);
            Put(b.Y);
        }

        var reactions = new Vector2d[system.Pins.Count];
        system.GetPinReactions(state.Time, y, reactions);
        for (var p = 0; p < reactions.Length; p++)
        {
            var at = system.WorldPoint(y, system.Pins[p].A);
            Put(reactions[p].X);
            Put(reactions[p].Y);
            Put(at.X);
            Put(at.Y);
        }

        var torques = new double[system.Actuators.Count];
        system.GetActuatorTorques(state.Time, torques);
        foreach (var torque in torques)
        {
            Put(torque);
        }

        var limitTorques = new double[system.Pins.Count];
        system.GetLimitTorques(y, limitTorques);
        for (var p = 0; p < limitTorques.Length; p++)
        {
            if (system.Pins[p].Limit is not null)
            {
                Put(limitTorques[p]);
            }
        }

        var loadForces = new Vector2d[system.Loads.Count];
        system.GetLoadForces(state.Time, loadForces);
        for (var l = 0; l < loadForces.Length; l++)
        {
            var at = system.WorldPoint(y, system.Loads[l].Attachment);
            Put(loadForces[l].X);
            Put(loadForces[l].Y);
            Put(at.X);
            Put(at.Y);
        }

        if (system.GroundLine is not null)
        {
            Put(system.GetGroundHeight(y));
        }

        var contactForces = new Vector2d[system.ContactCount];
        system.GetContactForces(state.Time, y, contactForces);
        for (var k = 0; k < contactForces.Length; k++)
        {
            var at = system.GetContactPoint(y, k);
            Put(contactForces[k].Y);
            Put(contactForces[k].X);
            Put(at.X);
            Put(at.Y);
        }
    }

    /// <summary>Running extremes of the per-step diagnostics.</summary>
    private sealed class Tracker
    {
        private readonly PlanarSystem system;
        private readonly double[] compressions;
        private readonly double[] minCompression;
        private readonly double[] maxCompression;
        private readonly double[] tensions;
        private readonly double[] minTension;
        private readonly double[] maxTension;
        private double maxBalanceError;
        private double maxPinPosition;
        private double maxPinVelocity;

        public Tracker(PlanarSystem system)
        {
            this.system = system;
            compressions = new double[system.BodyCount];
            minCompression = Enumerable.Repeat(double.PositiveInfinity, system.BodyCount).ToArray();
            maxCompression = Enumerable.Repeat(double.NegativeInfinity, system.BodyCount).ToArray();
            tensions = new double[system.Cables.Count];
            minTension = Enumerable.Repeat(double.PositiveInfinity, tensions.Length).ToArray();
            maxTension = Enumerable.Repeat(double.NegativeInfinity, tensions.Length).ToArray();
        }

        public void Observe(double time, double[] state, double initialEnergy)
        {
            maxBalanceError = Math.Max(maxBalanceError, Math.Abs(system.TotalEnergy(state) - initialEnergy));
            maxPinPosition = Math.Max(maxPinPosition, system.PinPositionResidual(state));
            maxPinVelocity = Math.Max(maxPinVelocity, system.PinVelocityResidual(state));

            system.GetMidpointCompressions(time, state, compressions);
            for (var b = 0; b < compressions.Length; b++)
            {
                minCompression[b] = Math.Min(minCompression[b], compressions[b]);
                maxCompression[b] = Math.Max(maxCompression[b], compressions[b]);
            }

            system.GetCableTensions(state, tensions);
            for (var c = 0; c < tensions.Length; c++)
            {
                minTension[c] = Math.Min(minTension[c], tensions[c]);
                maxTension[c] = Math.Max(maxTension[c], tensions[c]);
            }
        }

        public void Fill(RunDiagnostics diagnostics)
        {
            diagnostics.MaxEnergyBalanceError = maxBalanceError;
            diagnostics.MaxPinPositionResidual = maxPinPosition;
            diagnostics.MaxPinVelocityResidual = maxPinVelocity;
            for (var b = 0; b < system.BodyCount; b++)
            {
                var link = system.Links[b];
                var isStrut = link.Role == LinkRole.Strut;
                diagnostics.AxialForces.Add(new LinkAxialForce
                {
                    Link = link.Id,
                    Role = isStrut ? LinkRoles.Strut : LinkRoles.Link,
                    MinCompression = minCompression[b],
                    MaxCompression = maxCompression[b],
                });
                if (isStrut && minCompression[b] < -CompressionTolerance)
                {
                    diagnostics.StrutsInTension.Add(link.Id);
                }
            }

            for (var c = 0; c < tensions.Length; c++)
            {
                diagnostics.CableTensions.Add(new CableTensionRange
                {
                    Cable = system.Cables[c].Id,
                    MinTension = minTension[c],
                    MaxTension = maxTension[c],
                });
            }
        }
    }
}
