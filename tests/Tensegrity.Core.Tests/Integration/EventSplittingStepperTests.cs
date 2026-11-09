using Tensegrity.Core.Integration;

namespace Tensegrity.Core.Tests.Integration;

public class EventSplittingStepperTests
{
    /// <summary>
    /// x″ = −1 for x ≥ 0 and +1 for x &lt; 0: a particle in a V-shaped
    /// potential. Each piece is quadratic in time, which RK4 integrates
    /// exactly, so with the steps split at x = 0 the only error is rounding
    /// and the switch-time bracket. From x = 1 at rest it crosses zero at
    /// t = √2·(2j + 1) with speed ∓√2, period 4√2.
    /// </summary>
    private sealed class VPotential : ISwitchedSystem
    {
        private bool positive;

        public int EvaluationCount { get; private set; }

        public void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative)
        {
            EvaluationCount++;
            derivative[0] = state[1];
            derivative[1] = positive ? -1.0 : 1.0;
        }

        public void LatchModes(double time, ReadOnlySpan<double> state) => positive = state[0] >= 0;

        public bool ModesMatch(double time, ReadOnlySpan<double> state) => (state[0] >= 0) == positive;

        public static (double X, double V) Exact(double t)
        {
            var period = 4 * Math.Sqrt(2);
            var tau = t - (Math.Floor(t / period) * period);
            var half = 2 * Math.Sqrt(2);
            if (tau < half)
            {
                // Out from x = 1 to x = −1 through the negative well and back is symmetric.
                var s = tau;
                return s <= Math.Sqrt(2) ? (1 - (0.5 * s * s), -s) : (-1 + (0.5 * (half - s) * (half - s)), -(half - s));
            }

            var (x, v) = Exact(tau - half);
            return (-x, -v);
        }
    }

    /// <summary>Never agrees with its latch: every step chatters.</summary>
    private sealed class AlwaysSwitching : ISwitchedSystem
    {
        public void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative) => derivative.Clear();

        public void LatchModes(double time, ReadOnlySpan<double> state)
        {
        }

        public bool ModesMatch(double time, ReadOnlySpan<double> state) => false;
    }

    [Fact]
    public void SplitStepsFollowAPiecewiseQuadraticExactly()
    {
        var system = new VPotential();
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var state = new SystemState([1.0, 0.0]);
        var worst = 0.0;

        // dt does not divide the crossing times, so every crossing falls mid-step.
        FixedStepRunner.Run(state, stepper, 0.01, 3000, (_, s) =>
        {
            var (x, v) = VPotential.Exact(s.Time);
            worst = Math.Max(worst, Math.Max(Math.Abs(s.Values[0] - x), Math.Abs(s.Values[1] - v)));
        });

        // 30 s holds 11 crossings, at √2·(2j + 1) for j = 0..10.
        Assert.Equal(11, stepper.EventCount);
        Assert.Equal(0, stepper.UnresolvedSteps);

        // The only error left is landing up to TimeTolerance (1e-12 s) past
        // each switch with the old acceleration: ≤ 2e-12 m/s per crossing,
        // which then grows linearly in position, so ≈ 11 · 2e-12 · 30 s.
        Assert.InRange(worst, 0.0, 2e-10);
    }

    [Fact]
    public void WithoutSplittingTheSameProblemIsFarLessAccurate()
    {
        // Control for the test above: re-latching at every stage evaluates
        // the switched law directly, as plain RK4 would.
        var system = new VPotential();
        var state = new SystemState([1.0, 0.0]);
        var worst = 0.0;
        void Direct(double t, ReadOnlySpan<double> y, Span<double> dy)
        {
            system.LatchModes(t, y);
            system.Evaluate(t, y, dy);
        }

        FixedStepRunner.Run(state, new Rk4Integrator(), Direct, 0.01, 3000, (_, s) =>
        {
            var (x, v) = VPotential.Exact(s.Time);
            worst = Math.Max(worst, Math.Max(Math.Abs(s.Values[0] - x), Math.Abs(s.Values[1] - v)));
        });

        Assert.True(worst > 1e-6, $"Unsplit error {worst} was expected to be large.");
    }

    [Fact]
    public void StepWithNoSwitchIsASingleIntegratorStep()
    {
        var system = new VPotential();
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var state = new SystemState([1.0, 0.0]);

        stepper.Step(state, 0.1);

        Assert.Equal(4, system.EvaluationCount);
        Assert.Equal(0, stepper.EventCount);
        Assert.Equal(0.1, state.Time, 15);
        Assert.Equal(1 - 0.005, state.Values[0], 15);
    }

    [Fact]
    public void ChatteringIsCappedAndCounted()
    {
        var stepper = new EventSplittingStepper(new Rk4Integrator(), new AlwaysSwitching());
        var state = new SystemState([0.0]);

        stepper.Step(state, 0.01);

        Assert.Equal(EventSplittingStepper.MaxEventsPerStep, stepper.EventCount);
        Assert.Equal(1, stepper.UnresolvedSteps);
        Assert.Equal(0.01, state.Time, 15);
    }

    [Fact]
    public void RejectsBadArguments()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new EventSplittingStepper(new Rk4Integrator(), new VPotential(), 0));
        var stepper = new EventSplittingStepper(new Rk4Integrator(), new VPotential());
        Assert.Throws<ArgumentOutOfRangeException>(() => stepper.Step(new SystemState([1.0, 0.0]), -1));
    }
}
