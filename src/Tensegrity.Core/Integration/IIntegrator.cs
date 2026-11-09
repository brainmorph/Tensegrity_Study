namespace Tensegrity.Core.Integration;

/// <summary>
/// Advances a <see cref="SystemState"/> by one step. Implementations work on
/// the generic state vector only, so element code never depends on the
/// method (PLAN.md 4.2). Adaptive or implicit methods can be added later
/// behind this interface.
/// </summary>
public interface IIntegrator
{
    /// <summary>Short identifier recorded in run manifests, e.g. "rk4".</summary>
    string Name { get; }

    /// <summary>Advances <paramref name="state"/> in place from t to t + dt.</summary>
    void Step(SystemState state, double dt, Derivatives derivatives);
}
