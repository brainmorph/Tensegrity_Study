using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Output;

/// <summary>
/// Tests 12 (determinism) and 13 (round trip) from PLAN.md section 7, and
/// the contents of the run manifest.
/// </summary>
public sealed class RunRecorderTests : IDisposable
{
    private static readonly RunMetadata Metadata = new("planar_x", "test run", 42, "2026-11-09T00:00:00Z", "test", false);
    private static readonly RunSettings Settings = new(1e-4, 2_000, 50);

    private readonly string root = Directory.CreateTempSubdirectory("tensegrity-runs-").FullName;

    public void Dispose() => Directory.Delete(root, recursive: true);

    private string RunDir(string name) => Path.Combine(root, name);

    private static RunManifest RecordKickedX(string directory)
    {
        var model = PlanarX.Pinned();
        return RunRecorder.Record(model.System, PlanarX.KickedPinned(model), Settings, Metadata, directory);
    }

    [Fact]
    public void SameInputGivesByteIdenticalRuns()
    {
        RecordKickedX(RunDir("a"));
        RecordKickedX(RunDir("b"));

        Assert.Equal(File.ReadAllBytes(Path.Combine(RunDir("a"), RunFiles.Frames)), File.ReadAllBytes(Path.Combine(RunDir("b"), RunFiles.Frames)));
        Assert.Equal(File.ReadAllText(Path.Combine(RunDir("a"), RunFiles.Manifest)), File.ReadAllText(Path.Combine(RunDir("b"), RunFiles.Manifest)));
    }

    [Fact]
    public void FramesReadBackAsTheFloat32CastOfTheSimulatedState()
    {
        RecordKickedX(RunDir("run"));
        var run = RunReader.Open(RunDir("run"));

        // Re-simulate independently and compare each recorded frame exactly.
        var model = PlanarX.Pinned();
        var system = model.System;
        var state = PlanarX.KickedPinned(model);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var frame = 0;
        var tensions = new double[system.Cables.Count];
        var reactions = new Vector2d[system.Pins.Count];
        FixedStepRunner.Run(state, stepper, Settings.Dt, Settings.Steps, (step, s) =>
        {
            if (step % Settings.FrameStride != 0)
            {
                return;
            }

            Assert.Equal((float)s.Time, run.Value(frame, FrameChannels.Time));
            for (var b = 0; b < system.BodyCount; b++)
            {
                var id = system.Links[b].Id;
                var pose = system.GetPose(s.Values, b);
                Assert.Equal((float)pose.Position.X, run.Value(frame, FrameChannels.LinkX(id)));
                Assert.Equal((float)pose.Position.Y, run.Value(frame, FrameChannels.LinkY(id)));
                Assert.Equal((float)pose.Angle, run.Value(frame, FrameChannels.LinkTheta(id)));
            }

            system.GetCableTensions(s.Values, tensions);
            for (var c = 0; c < tensions.Length; c++)
            {
                var cable = system.Cables[c];
                Assert.Equal((float)tensions[c], run.Value(frame, FrameChannels.CableTension(cable.Id)));
                Assert.Equal((float)system.WorldPoint(s.Values, cable.A).X, run.Value(frame, FrameChannels.CableAx(cable.Id)));
                Assert.Equal((float)system.WorldPoint(s.Values, cable.B).Y, run.Value(frame, FrameChannels.CableBy(cable.Id)));
            }

            system.GetPinReactions(s.Time, s.Values, reactions);
            for (var p = 0; p < reactions.Length; p++)
            {
                var pin = system.Pins[p];
                Assert.Equal((float)reactions[p].X, run.Value(frame, FrameChannels.PinFx(pin.Id)));
                Assert.Equal((float)reactions[p].Y, run.Value(frame, FrameChannels.PinFy(pin.Id)));
                Assert.Equal((float)system.WorldPoint(s.Values, pin.A).X, run.Value(frame, FrameChannels.PinX(pin.Id)));
            }

            frame++;
        });

        Assert.Equal(frame, run.FrameCount);
    }

    [Fact]
    public void ManifestDescribesTheRun()
    {
        var manifest = RecordKickedX(RunDir("run"));
        var read = RunReader.Open(RunDir("run")).Manifest;

        Assert.Equal(2_000 / 50 + 1, manifest.FrameCount);
        Assert.Equal(manifest.FrameCount, read.FrameCount);
        Assert.Equal("rk4", read.Integrator);
        Assert.Equal(RunRecorder.ConstraintMethod, read.ConstraintMethod);
        Assert.Equal(0.2, read.Duration, 15);
        Assert.Equal(42, read.Seed);
        Assert.Equal(1 + (2 * 3) + (3 * 5) + (2 * 4), read.FrameLayout.Channels.Count);
        Assert.Equal(manifest.FrameCount, read.Diagnostics.Energy.Total.Count);
        Assert.Equal(["s1", "s2"], read.Diagnostics.AxialForces.Select(a => a.Link));
        Assert.All(read.Diagnostics.AxialForces, a => Assert.True(a.MinCompression > 10, $"{a.Link} min compression {a.MinCompression}"));
        Assert.Empty(read.Diagnostics.StrutsInTension);
        Assert.All(read.Diagnostics.CableTensions, c => Assert.True(c.MinTension > 0));
        Assert.InRange(read.Diagnostics.MaxPinPositionResidual, 0.0, 1e-13);
        Assert.InRange(read.Diagnostics.MaxEnergyBalanceError, 0.0, 1e-9);
        Assert.Equal(0, read.Diagnostics.EventCount);
    }

    [Fact]
    public void ManifestTopologyRebuildsTheInitialSystemExactly()
    {
        var manifest = RecordKickedX(RunDir("run"));
        var model = PlanarX.Pinned();
        var initial = PlanarX.KickedPinned(model);

        var (system, state) = TopologyMapping.FromDocument(RunReader.Open(RunDir("run")).Manifest.Topology);

        Assert.Equal(initial.Values, state);
        Assert.Equal(
            RunFiles.SerializeTopology(manifest.Topology),
            RunFiles.SerializeTopology(TopologyMapping.ToDocument(system, state, manifest.Topology.Name, manifest.Topology.Description)));
        Assert.Equal(model.System.Cables.Select(c => c.RestLength), system.Cables.Select(c => c.RestLength));
        Assert.All(system.Links, l => Assert.Equal(LinkRole.Strut, l.Role));
    }

    [Fact]
    public void CableEventsAreCountedLikeTheRegressionScenario()
    {
        var system = CableOscillator.CreateSystem(CableOscillatorScenario.Damping);
        var theory = CableOscillator.CreateTheory(CableOscillatorScenario.Damping);
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - (3 * theory.StaticStretch), 0.0);

        var manifest = RunRecorder.Record(system, state, new RunSettings(1e-4, 20_000, 1_000), Metadata with { TopologyName = "cable_oscillator" }, RunDir("run"));

        Assert.Equal(9, manifest.Diagnostics.EventCount);
        Assert.Equal(0, manifest.Diagnostics.UnresolvedEventSteps);
        Assert.Equal(0.0, manifest.Diagnostics.CableTensions[0].MinTension);
    }

    [Fact]
    public void AStrutInTensionIsFlagged()
    {
        // A bar marked as a strut and hung from a pin is in tension: a
        // configuration error the manifest must report.
        var bar = PlanarLink.SlenderBar("hanger", 1.0, 1.0, LinkRole.Strut);
        var pin = new PinJoint("pivot", Attachment.OnLink("hanger", bar.EndA), Attachment.Ground(Vector2d.Zero));
        var system = new PlanarSystem([bar], PlanarSystem.StandardGravity, [pin]);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, -0.5), -Math.PI / 2), default);

        var manifest = RunRecorder.Record(system, state, new RunSettings(1e-3, 10, 5), Metadata, RunDir("run"));

        Assert.Equal(["hanger"], manifest.Diagnostics.StrutsInTension);
        Assert.Equal(-0.5 * 9.81, manifest.Diagnostics.AxialForces[0].MaxCompression, 1e-9);
    }

    [Fact]
    public void RefusesToOverwriteARun()
    {
        RecordKickedX(RunDir("run"));

        Assert.Throws<IOException>(() => RecordKickedX(RunDir("run")));
    }

    [Fact]
    public void SettingsForDurationNeedAWholeNumberOfSteps()
    {
        Assert.Equal(20_000, RunSettings.ForDuration(1e-4, 2.0, 100).Steps);
        Assert.Throws<ArgumentException>(() => RunSettings.ForDuration(1e-4, 2.00005, 100));
        Assert.Throws<ArgumentOutOfRangeException>(() => RunSettings.ForDuration(0, 1, 1));
    }

    [Fact]
    public void MappingRejectsMalformedDocuments()
    {
        TopologyDocument Valid() => TopologyMapping.ToDocument(PlanarX.Pinned().System, PlanarX.Pinned().InitialState, "x");

        var both = Valid();
        both.Pins[0].A.Ground = [0, 0];
        var role = Valid();
        role.Links[0].Role = "beam";
        var gravity = Valid();
        gravity.Gravity = [0, -9.81, 0];

        Assert.Throws<FormatException>(() => TopologyMapping.FromDocument(both));
        Assert.Throws<FormatException>(() => TopologyMapping.FromDocument(role));
        Assert.Throws<FormatException>(() => TopologyMapping.FromDocument(gravity));
    }
}
