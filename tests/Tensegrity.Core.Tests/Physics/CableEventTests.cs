using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 8 (PLAN.md section 7): a cable that goes slack and taut again. With
/// steps split at each switch, RK4 keeps its fourth order across events.
/// </summary>
public class CableEventTests
{
    /// <summary>Three times the static stretch: the cable spends most of each period slack.</summary>
    private const double AmplitudeOverStretch = 3.0;

    private static (double Y, double Vy, long Events) Simulate(double damping, double dt, double duration, bool split)
    {
        var system = CableOscillator.CreateSystem(damping);
        var theory = CableOscillator.CreateTheory(damping);
        var amplitude = AmplitudeOverStretch * theory.StaticStretch;
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - amplitude, 0.0);
        var steps = (long)Math.Round(duration / dt);
        long events = 0;

        if (split)
        {
            var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
            FixedStepRunner.Run(state, stepper, dt, steps);
            events = stepper.EventCount;
        }
        else
        {
            // Control: the physical law evaluated at every stage, as plain RK4 would.
            void Direct(double t, ReadOnlySpan<double> y, Span<double> dy)
            {
                system.LatchModes(t, y);
                system.Evaluate(t, y, dy);
            }

            FixedStepRunner.Run(state, new Rk4Integrator(), Direct, dt, steps);
        }

        return (system.GetPose(state.Values, 0).Position.Y, system.GetTwist(state.Values, 0).Linear.Y, events);
    }

    [Fact]
    public void BouncingMatchesThePiecewiseClosedForm()
    {
        var system = CableOscillator.CreateSystem();
        var theory = CableOscillator.CreateTheory();
        var amplitude = AmplitudeOverStretch * theory.StaticStretch;
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - amplitude, 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var worst = 0.0;
        const double duration = 2.0;

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000, (_, s) =>
        {
            var (y, vy) = theory.Bouncing(amplitude, s.Time);
            worst = Math.Max(worst, Math.Max(
                Math.Abs(system.GetPose(s.Values, 0).Position.Y - y),
                Math.Abs(system.GetTwist(s.Values, 0).Linear.Y - vy)));
        });

        // Two switches per bounce: taut → slack on the way up, slack → taut on the way down.
        var period = theory.BouncePeriod(amplitude);
        var t1 = theory.TimeToSlack(amplitude);
        var expectedEvents = 0;
        for (var k = 0; (k * period) + t1 < duration; k++)
        {
            expectedEvents++;
            if ((k * period) + period - t1 < duration)
            {
                expectedEvents++;
            }
        }

        Assert.True(expectedEvents >= 6, $"Only {expectedEvents} switches in the run.");
        Assert.Equal(expectedEvents, stepper.EventCount);
        Assert.Equal(0, stepper.UnresolvedSteps);
        Assert.InRange(worst, 0.0, 1e-10);
    }

    [Fact]
    public void BouncingConvergesAtFourthOrderAcrossEvents()
    {
        var theory = CableOscillator.CreateTheory();
        var amplitude = AmplitudeOverStretch * theory.StaticStretch;
        const double duration = 1.0;
        var exact = theory.Bouncing(amplitude, duration);
        double ErrorAt(double dt)
        {
            var (y, vy, _) = Simulate(0, dt, duration, split: true);
            return Math.Abs(y - exact.Y) + Math.Abs(vy - exact.Vy);
        }

        // ω·dt runs from 0.032 to 0.004: the asymptotic range. Larger steps
        // (ω·dt ≳ 0.1) are still pre-asymptotic and give ratios near 10.
        double[] errors = [ErrorAt(1e-3), ErrorAt(5e-4), ErrorAt(2.5e-4), ErrorAt(1.25e-4)];

        for (var i = 0; i + 1 < errors.Length; i++)
        {
            Assert.InRange(errors[i] / errors[i + 1], 13.0, 19.0);
        }
    }

    [Fact]
    public void WithoutSplittingTheOrderIsLost()
    {
        // Control for the test above: same problem, steps not split. The
        // unsplit error is erratic (it depends on where each switch falls in
        // its step), so compare over two halvings rather than one.
        var theory = CableOscillator.CreateTheory();
        var amplitude = AmplitudeOverStretch * theory.StaticStretch;
        const double duration = 1.0;
        var exact = theory.Bouncing(amplitude, duration);
        double ErrorAt(double dt, bool split)
        {
            var (y, vy, _) = Simulate(0, dt, duration, split);
            return Math.Abs(y - exact.Y) + Math.Abs(vy - exact.Vy);
        }

        var unsplitCoarse = ErrorAt(1e-3, false);
        var unsplitFine = ErrorAt(2.5e-4, false);

        // Fourth order would give 256 over two halvings.
        Assert.True(unsplitCoarse / unsplitFine < 16, $"Unsplit error fell by {unsplitCoarse / unsplitFine} over two halvings.");
        Assert.True(unsplitFine > 1000 * ErrorAt(2.5e-4, true));
    }

    /// <summary>
    /// With damping the tension jumps by c·L̇ when the cable catches, and it
    /// can also unload while still stretched (recoiling). There is no closed
    /// form, so convergence is checked on successive differences.
    /// </summary>
    [Fact]
    public void DampedBouncingConvergesAtFourthOrder()
    {
        const double damping = 8.0;
        const double duration = 1.0;
        double[] dts = [2e-3, 1e-3, 5e-4, 2.5e-4];
        var results = dts.Select(dt => Simulate(damping, dt, duration, split: true)).ToArray();

        Assert.All(results, r => Assert.True(r.Events >= 4, $"Only {r.Events} switches."));
        for (var i = 0; i + 2 < results.Length; i++)
        {
            var ratio = Math.Abs(results[i].Y - results[i + 1].Y) / Math.Abs(results[i + 1].Y - results[i + 2].Y);
            Assert.InRange(ratio, 13.0, 19.0);
        }
    }
}
