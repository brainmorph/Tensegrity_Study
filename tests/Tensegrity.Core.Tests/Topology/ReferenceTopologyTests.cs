using Tensegrity.Core.Integration;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Topology;

/// <summary>
/// The reference topology catalog and JSON topology files. Each catalog
/// entry must set up exactly what its regression scenario simulates, so a
/// CLI run of the exported file reproduces what the baselines protect.
/// </summary>
public sealed class ReferenceTopologyTests : IDisposable
{
    private readonly string directory = Directory.CreateTempSubdirectory("tensegrity-topology-").FullName;

    public void Dispose() => Directory.Delete(directory, recursive: true);

    public static TheoryData<string> Names() => new(ReferenceTopologies.Names);

    /// <summary>The system and initial state each regression scenario builds for itself.</summary>
    private static (PlanarSystem System, double[] State) ScenarioSetup(string name)
    {
        switch (name)
        {
            case "pendulum":
            {
                var theory = PendulumScenario.CreateTheory();
                var system = PendulumScenario.CreateSystem();
                return (system, PendulumScenario.CreateState(system, theory, theory.HangingAngle + PendulumScenario.Amplitude, 0.0).Values);
            }

            case "cable_oscillator":
            {
                var system = CableOscillator.CreateSystem(CableOscillatorScenario.Damping);
                var theory = CableOscillator.CreateTheory(CableOscillatorScenario.Damping);
                var y = theory.EquilibriumY - (CableOscillatorScenario.AmplitudeOverStretch * theory.StaticStretch);
                return (system, CableOscillator.CreateState(system, y, 0.0).Values);
            }

            case "planar_x":
            {
                var model = PlanarX.Pinned();
                return (model.System, PlanarX.KickedPinned(model).Values);
            }

            case "planar_x_rest":
            {
                var model = PlanarX.Pinned();
                return (model.System, model.InitialState.ToArray());
            }

            case "planar_x_free":
            {
                var model = PlanarX.Free();
                return (model.System, PlanarX.PerturbedFree(model).Values);
            }

            default:
                throw new ArgumentException(name);
        }
    }

    [Theory]
    [MemberData(nameof(Names))]
    public void CatalogEntryMatchesItsScenarioBitForBit(string name)
    {
        var model = ReferenceTopologies.Create(name);
        var (system, state) = ScenarioSetup(name);

        Assert.Equal(name, model.Name);
        Assert.Equal(state, model.InitialState.ToArray());
        Assert.Equal(
            RunFiles.SerializeTopology(TopologyMapping.ToDocument(system, state, name)),
            RunFiles.SerializeTopology(TopologyMapping.ToDocument(model.System, model.InitialState, name)));
    }

    [Theory]
    [MemberData(nameof(Names))]
    public void SavedFileLoadsBackToTheSameModel(string name)
    {
        var model = ReferenceTopologies.Create(name);
        var path = Path.Combine(directory, name + ".json");

        TopologyFiles.Save(model, path);
        var loaded = TopologyFiles.Load(path);

        Assert.Equal(model.InitialState.ToArray(), loaded.InitialState.ToArray());
        Assert.Equal(File.ReadAllText(path), TopologyFiles.Serialize(loaded));
        Assert.Equal(model.Simulation!.Dt, loaded.Simulation!.Dt);
        Assert.Equal(model.Description, loaded.Description);
    }

    [Fact]
    public void LoadedFileSimulatesExactlyLikeTheGenerator()
    {
        var model = ReferenceTopologies.PlanarX();
        var path = Path.Combine(directory, "x.json");
        TopologyFiles.Save(model, path);
        var loaded = TopologyFiles.Load(path);
        double[] Simulate(PlanarModel m)
        {
            var state = m.CreateInitialState();
            FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), m.System), 1e-4, 2_000);
            return state.Values;
        }

        Assert.Equal(Simulate(model), Simulate(loaded));
    }

    [Fact]
    public void MalformedFilesAreReportedWithTheirPath()
    {
        var path = Path.Combine(directory, "bad.json");
        var document = TopologyFiles.ToDocument(ReferenceTopologies.PlanarXRest());
        document.Cables[0].RestLength = -1;
        File.WriteAllText(path, RunFiles.SerializeTopology(document));

        var error = Assert.Throws<InvalidDataException>(() => TopologyFiles.Load(path));
        Assert.Contains(path, error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void UnknownCatalogNameIsAnError() =>
        Assert.Throws<ArgumentException>(() => ReferenceTopologies.Create("planar_y"));
}
