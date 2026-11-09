namespace Tensegrity.Core.Integration;

/// <summary>
/// Runs an integrator for a fixed number of equal steps. Time is set to
/// t0 + k·dt after step k rather than accumulated by repeated addition,
/// so it does not drift over long runs.
/// </summary>
public static class FixedStepRunner
{
    /// <summary>
    /// Advances <paramref name="state"/> by <paramref name="steps"/> steps of
    /// <paramref name="dt"/>. If given, <paramref name="observer"/> is called
    /// with (step index, state) at step 0 and after every
    /// <paramref name="stride"/>-th step. The observer must not modify the state.
    /// </summary>
    public static void Run(
        SystemState state,
        IIntegrator integrator,
        Derivatives derivatives,
        double dt,
        long steps,
        Action<long, SystemState>? observer = null,
        long stride = 1)
    {
        ArgumentNullException.ThrowIfNull(integrator);
        ArgumentNullException.ThrowIfNull(derivatives);
        Run(state, (s, h) => integrator.Step(s, h, derivatives), dt, steps, observer, stride);
    }

    /// <summary>
    /// As above, for a switched system stepped by an
    /// <see cref="EventSplittingStepper"/>, which splits steps at mode switches.
    /// </summary>
    public static void Run(
        SystemState state,
        EventSplittingStepper stepper,
        double dt,
        long steps,
        Action<long, SystemState>? observer = null,
        long stride = 1)
    {
        ArgumentNullException.ThrowIfNull(stepper);
        Run(state, stepper.Step, dt, steps, observer, stride);
    }

    private static void Run(
        SystemState state,
        Action<SystemState, double> step,
        double dt,
        long steps,
        Action<long, SystemState>? observer,
        long stride)
    {
        ArgumentNullException.ThrowIfNull(state);
        ArgumentOutOfRangeException.ThrowIfNegative(steps);
        ArgumentOutOfRangeException.ThrowIfLessThan(stride, 1);

        var t0 = state.Time;
        observer?.Invoke(0, state);
        for (long k = 1; k <= steps; k++)
        {
            step(state, dt);
            state.Time = t0 + (k * dt);
            if (observer is not null && k % stride == 0)
            {
                observer(k, state);
            }
        }
    }
}
