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
        ArgumentNullException.ThrowIfNull(state);
        ArgumentNullException.ThrowIfNull(integrator);
        ArgumentOutOfRangeException.ThrowIfNegative(steps);
        ArgumentOutOfRangeException.ThrowIfLessThan(stride, 1);

        var t0 = state.Time;
        observer?.Invoke(0, state);
        for (long k = 1; k <= steps; k++)
        {
            integrator.Step(state, dt, derivatives);
            state.Time = t0 + (k * dt);
            if (observer is not null && k % stride == 0)
            {
                observer(k, state);
            }
        }
    }
}
