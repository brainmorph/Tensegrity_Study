using Tensegrity.Core.Integration;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Topology;

/// <summary>
/// A ready-to-run planar structure: the system, its initial state, and
/// optionally its default run settings. Built by a generator or loaded from
/// a JSON topology file (<see cref="TopologyFiles"/>); both give the same
/// thing.
/// </summary>
public sealed class PlanarModel
{
    private readonly double[] initialState;

    public PlanarModel(string name, PlanarSystem system, double[] initialState, string? description = null, SimulationDocument? simulation = null)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(name);
        ArgumentNullException.ThrowIfNull(system);
        ArgumentNullException.ThrowIfNull(initialState);
        if (initialState.Length != system.StateLength)
        {
            throw new ArgumentException($"Initial state length {initialState.Length} does not match the system's {system.StateLength}.", nameof(initialState));
        }

        Name = name;
        System = system;
        this.initialState = (double[])initialState.Clone();
        Description = description;
        Simulation = simulation;
    }

    public string Name { get; }

    public string? Description { get; }

    /// <summary>Default dt, duration and frame stride, if the model carries them.</summary>
    public SimulationDocument? Simulation { get; }

    public PlanarSystem System { get; }

    public ReadOnlySpan<double> InitialState => initialState;

    /// <summary>A fresh, independent copy of the initial state at t = 0.</summary>
    public SystemState CreateInitialState() => new((double[])initialState.Clone());

    /// <summary>The same model with a different initial state, name, description or run settings.</summary>
    public PlanarModel With(
        double[]? initialState = null,
        string? name = null,
        string? description = null,
        SimulationDocument? simulation = null) =>
        new(name ?? Name, System, initialState ?? this.initialState, description ?? Description, simulation ?? Simulation);
}
