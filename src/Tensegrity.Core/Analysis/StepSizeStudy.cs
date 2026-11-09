using Tensegrity.Core.Integration;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Analysis;

/// <summary>One step size in a <see cref="StepSizeStudy"/>.</summary>
/// <param name="Dt">Step size, s.</param>
/// <param name="ChangeToNext">
/// Largest difference in any position or angle from the next finer level,
/// over the sample times; an estimate of this level's error. Null for the
/// finest level.
/// </param>
/// <param name="Ratio">
/// <see cref="ChangeToNext"/> of the previous (coarser) level divided by
/// this one: ≈ 16 once RK4 is in its asymptotic range. Null where either is
/// missing.
/// </param>
/// <param name="EnergyBalanceError">Worst |E_total(t) − E_total(0)| over every step, J.</param>
/// <param name="Events">Steps split at a cable mode switch.</param>
public sealed record StepSizeLevel(double Dt, double? ChangeToNext, double? Ratio, double EnergyBalanceError, long Events);

/// <summary>
/// Test 14 (PLAN.md section 7): runs one topology at dt, dt/2, dt/4, … and
/// reports how much the trajectory changes from each level to the next.
/// Every level is sampled at the same times, so the comparison needs no
/// interpolation. A step size is converged when halving it changes the
/// result by less than the accuracy the study needs, and is in RK4's
/// asymptotic range when the change falls by ≈ 16 per halving.
/// </summary>
public static class StepSizeStudy
{
    /// <summary>Sample times per run, evenly spaced, including the end (t = 0 is identical at every level).</summary>
    public const int Samples = 10;

    public static IReadOnlyList<StepSizeLevel> Run(
        PlanarSystem system,
        ReadOnlySpan<double> initialState,
        double coarsestDt,
        double duration,
        int levels,
        double eventTolerance = EventSplittingStepper.DefaultTimeTolerance)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentOutOfRangeException.ThrowIfLessThan(levels, 2);
        var coarseSteps = Math.Round(duration / coarsestDt);
        if (!(coarseSteps >= Samples) || Math.Abs((coarseSteps * coarsestDt) - duration) > 1e-9 * duration || coarseSteps % Samples != 0)
        {
            throw new ArgumentException(
                $"Duration {duration} s must be a whole number of {coarsestDt} s steps, divisible by {Samples}.", nameof(duration));
        }

        var initial = initialState.ToArray();
        var n = system.CoordinateCount;
        var positions = new List<double[][]>(levels);
        var balance = new double[levels];
        var events = new long[levels];

        for (var k = 0; k < levels; k++)
        {
            var dt = coarsestDt / (1L << k);
            var steps = (long)coarseSteps << k;
            var stride = steps / Samples;
            var state = new SystemState((double[])initial.Clone());
            var stepper = new EventSplittingStepper(new Rk4Integrator(), system, eventTolerance);
            var initialEnergy = system.TotalEnergy(state.Values);
            var samples = new double[Samples][];
            var worst = 0.0;

            FixedStepRunner.Run(state, stepper, dt, steps, (step, s) =>
            {
                worst = Math.Max(worst, Math.Abs(system.TotalEnergy(s.Values) - initialEnergy));
                if (step > 0 && step % stride == 0)
                {
                    samples[(step / stride) - 1] = s.Values[..n];
                }
            });

            positions.Add(samples);
            balance[k] = worst;
            events[k] = stepper.EventCount;
        }

        var change = new double?[levels];
        for (var k = 0; k + 1 < levels; k++)
        {
            var max = 0.0;
            for (var s = 0; s < Samples; s++)
            {
                for (var i = 0; i < n; i++)
                {
                    max = Math.Max(max, Math.Abs(positions[k][s][i] - positions[k + 1][s][i]));
                }
            }

            change[k] = max;
        }

        var result = new List<StepSizeLevel>(levels);
        for (var k = 0; k < levels; k++)
        {
            double? ratio = k > 0 && change[k - 1] is { } coarser && change[k] is { } finer && finer > 0 ? coarser / finer : null;
            result.Add(new StepSizeLevel(coarsestDt / (1L << k), change[k], ratio, balance[k], events[k]));
        }

        return result;
    }
}
