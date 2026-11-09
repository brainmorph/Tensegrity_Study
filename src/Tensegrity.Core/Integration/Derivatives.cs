namespace Tensegrity.Core.Integration;

/// <summary>
/// Right-hand side of a first-order ODE, dy/dt = f(t, y). Writes f into
/// <paramref name="derivative"/>, which has the same length as
/// <paramref name="state"/>. Must not keep references to either span.
/// </summary>
public delegate void Derivatives(double time, ReadOnlySpan<double> state, Span<double> derivative);
