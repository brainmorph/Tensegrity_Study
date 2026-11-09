namespace Tensegrity.Core.Integration;

/// <summary>
/// An ODE whose right-hand side has discrete modes, such as cables switching
/// between slack and taut. The modes are latched from a state and held fixed
/// while the integrator works, so each RK4 step sees a smooth right-hand
/// side; <see cref="EventSplittingStepper"/> finds where the true mode
/// changes and splits the step there (PLAN.md 4.2).
/// </summary>
public interface ISwitchedSystem
{
    /// <summary>dy/dt using the latched modes.</summary>
    void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative);

    /// <summary>Sets the latched modes to those of <paramref name="state"/>.</summary>
    void LatchModes(double time, ReadOnlySpan<double> state);

    /// <summary>True if every mode of <paramref name="state"/> equals the latched mode.</summary>
    bool ModesMatch(double time, ReadOnlySpan<double> state);
}
