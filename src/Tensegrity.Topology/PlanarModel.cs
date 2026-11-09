using Tensegrity.Core.Integration;
using Tensegrity.Core.Systems;

namespace Tensegrity.Topology;

/// <summary>
/// A ready-to-run planar structure: the system and its initial state. Built
/// by a generator; JSON topology files (M3) will produce the same thing.
/// </summary>
public sealed class PlanarModel
{
    private readonly double[] initialState;

    public PlanarModel(string name, PlanarSystem system, double[] initialState)
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
    }

    public string Name { get; }

    public PlanarSystem System { get; }

    public ReadOnlySpan<double> InitialState => initialState;

    /// <summary>A fresh, independent copy of the initial state at t = 0.</summary>
    public SystemState CreateInitialState() => new((double[])initialState.Clone());
}
