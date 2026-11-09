namespace Tensegrity.Core.Integration;

/// <summary>
/// Steps a <see cref="ISwitchedSystem"/> with an underlying fixed-step
/// integrator, splitting each step at mode switches so the integrator keeps
/// its order across them (PLAN.md 4.2):
/// <list type="number">
/// <item>Latch the modes at the start and take a trial step.</item>
/// <item>If any mode at the end differs, bisect the step length until the
/// switch is bracketed within <see cref="TimeTolerance"/>.</item>
/// <item>Advance to the end of the bracket, just past the switch, latch the
/// new modes, and continue with the rest of the step.</item>
/// </list>
/// Each trial is a single step from the start state, so the bisection works
/// on the integrator's own solution. A switch that happens and reverts
/// within one step is not seen; keep dt short against cable dynamics.
///
/// Not thread-safe: scratch state is reused between calls.
/// </summary>
public sealed class EventSplittingStepper
{
    /// <summary>Default bracket width for a switch time, s.</summary>
    public const double DefaultTimeTolerance = 1e-12;

    /// <summary>
    /// Switches resolved within one step before the rest of the step is
    /// taken unsplit. Reaching it means modes are chattering at a boundary,
    /// which is counted in <see cref="UnresolvedSteps"/>.
    /// </summary>
    public const int MaxEventsPerStep = 64;

    private readonly IIntegrator integrator;
    private readonly ISwitchedSystem system;
    private readonly Derivatives derivatives;
    private SystemState trial = new(0);

    public EventSplittingStepper(IIntegrator integrator, ISwitchedSystem system, double timeTolerance = DefaultTimeTolerance)
    {
        ArgumentNullException.ThrowIfNull(integrator);
        ArgumentNullException.ThrowIfNull(system);
        if (!double.IsFinite(timeTolerance) || timeTolerance <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(timeTolerance), timeTolerance, "Must be finite and positive.");
        }

        this.integrator = integrator;
        this.system = system;
        derivatives = system.Evaluate;
        TimeTolerance = timeTolerance;
    }

    /// <summary>Width of the bracket each switch time is located to, s.</summary>
    public double TimeTolerance { get; }

    /// <summary>Name of the underlying integrator, e.g. "rk4".</summary>
    public string IntegratorName => integrator.Name;

    /// <summary>Number of steps split at a mode switch so far.</summary>
    public long EventCount { get; private set; }

    /// <summary>Number of steps that hit <see cref="MaxEventsPerStep"/> and finished unsplit.</summary>
    public long UnresolvedSteps { get; private set; }

    /// <summary>Advances <paramref name="state"/> in place from t to t + dt.</summary>
    public void Step(SystemState state, double dt)
    {
        ArgumentNullException.ThrowIfNull(state);
        if (!double.IsFinite(dt) || dt <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "Must be finite and positive.");
        }

        if (trial.Length != state.Length)
        {
            trial = new SystemState(state.Length);
        }

        var remaining = dt;
        var events = 0;
        while (remaining > 0)
        {
            system.LatchModes(state.Time, state.Values);
            TrialStep(state, remaining);
            if (system.ModesMatch(trial.Time, trial.Values) || events >= MaxEventsPerStep)
            {
                if (events >= MaxEventsPerStep)
                {
                    UnresolvedSteps++;
                }

                Accept(state);
                return;
            }

            // The modes hold at h = 0 and fail at h = remaining: bisect.
            double lo = 0, hi = remaining;
            while (hi - lo > TimeTolerance)
            {
                var mid = 0.5 * (lo + hi);
                TrialStep(state, mid);
                if (system.ModesMatch(trial.Time, trial.Values))
                {
                    lo = mid;
                }
                else
                {
                    hi = mid;
                }
            }

            // Land just past the switch, so the next latch picks up the new mode.
            TrialStep(state, hi);
            Accept(state);
            remaining -= hi;
            events++;
            EventCount++;
        }
    }

    private void TrialStep(SystemState from, double h)
    {
        from.Values.CopyTo(trial.Values, 0);
        trial.Time = from.Time;
        integrator.Step(trial, h, derivatives);
    }

    private void Accept(SystemState state)
    {
        trial.Values.CopyTo(state.Values, 0);
        state.Time = trial.Time;
    }
}
