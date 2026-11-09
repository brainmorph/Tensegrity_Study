using Tensegrity.Core.Integration;

namespace Tensegrity.Core.Tests.Integration;

public class Rk4IntegratorTests
{
    /// <summary>Harmonic oscillator y'' = −y as a first-order system [y, y'].</summary>
    private static void Oscillator(double t, ReadOnlySpan<double> y, Span<double> dy)
    {
        dy[0] = y[1];
        dy[1] = -y[0];
    }

    private static double OscillatorError(double dt, double duration)
    {
        var steps = (long)Math.Round(duration / dt);
        var state = new SystemState([1.0, 0.0]);
        FixedStepRunner.Run(state, new Rk4Integrator(), Oscillator, dt, steps);

        // Exact: y = cos t, y' = −sin t.
        return Math.Max(Math.Abs(state.Values[0] - Math.Cos(duration)), Math.Abs(state.Values[1] + Math.Sin(duration)));
    }

    [Fact]
    public void SingleStepMatchesHandComputedRk4()
    {
        // y' = y from y = 1 with h = 0.1: RK4 gives the Taylor series of e^h to 4th order.
        var state = new SystemState([1.0]);
        new Rk4Integrator().Step(state, 0.1, (_, y, dy) => dy[0] = y[0]);

        const double h = 0.1;
        Assert.Equal(1 + h + (h * h / 2) + (h * h * h / 6) + (h * h * h * h / 24), state.Values[0], 15);
        Assert.Equal(0.1, state.Time);
    }

    [Fact]
    public void PassesStageTimesToDerivatives()
    {
        var times = new List<double>();
        new Rk4Integrator().Step(new SystemState([0.0], time: 2.0), 0.5, (t, _, dy) =>
        {
            times.Add(t);
            dy[0] = 0;
        });

        Assert.Equal([2.0, 2.25, 2.25, 2.5], times);
    }

    [Fact]
    public void IntegratesCubicInTimeExactly()
    {
        // y' = 3t² ⇒ y = t³. RK4 is exact for polynomials up to degree 4.
        var state = new SystemState([0.0]);
        FixedStepRunner.Run(state, new Rk4Integrator(), (t, _, dy) => dy[0] = 3 * t * t, 0.125, 16);

        Assert.Equal(8.0, state.Values[0], 13);
    }

    /// <summary>Test 5: halving dt cuts global error by ≈ 2⁴ = 16.</summary>
    [Fact]
    public void GlobalErrorIsFourthOrderOnHarmonicOscillator()
    {
        double[] dts = [0.1, 0.05, 0.025, 0.0125];
        var errors = dts.Select(dt => OscillatorError(dt, duration: 10.0)).ToArray();

        for (var i = 0; i + 1 < errors.Length; i++)
        {
            var ratio = errors[i] / errors[i + 1];
            Assert.InRange(ratio, 15.0, 17.0);
            Assert.InRange(Math.Log2(ratio), 3.9, 4.1);
        }
    }

    [Fact]
    public void RunnerSetsTimeWithoutAccumulatingRoundoff()
    {
        var state = new SystemState([0.0]);
        FixedStepRunner.Run(state, new Rk4Integrator(), (_, _, dy) => dy[0] = 0, 0.1, 1000);

        Assert.Equal(1000 * 0.1, state.Time);
    }

    [Fact]
    public void RunnerObservesStepZeroAndEveryStride()
    {
        var seen = new List<long>();
        FixedStepRunner.Run(new SystemState([0.0]), new Rk4Integrator(), (_, _, dy) => dy[0] = 0, 0.1, 10, (k, _) => seen.Add(k), stride: 4);

        Assert.Equal([0L, 4L, 8L], seen);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1e-3)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void RejectsInvalidStep(double dt)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new Rk4Integrator().Step(new SystemState([0.0]), dt, (_, _, dy) => dy[0] = 0));
    }

    [Fact]
    public void ReusedIntegratorHandlesChangingStateLength()
    {
        var rk4 = new Rk4Integrator();
        var small = new SystemState([1.0]);
        var large = new SystemState([1.0, 0.0]);

        rk4.Step(small, 0.1, (_, y, dy) => dy[0] = y[0]);
        rk4.Step(large, 0.1, Oscillator);

        Assert.Equal(Math.Cos(0.1), large.Values[0], 6);
    }

    [Fact]
    public void IdenticalRunsAreBitIdentical()
    {
        static double[] Run()
        {
            var state = new SystemState([0.3, -1.2]);
            FixedStepRunner.Run(state, new Rk4Integrator(), (_, y, dy) =>
            {
                dy[0] = y[1];
                dy[1] = -Math.Sin(y[0]) - (0.1 * y[1]);
            }, 1e-3, 5000);
            return state.Values;
        }

        var a = Run();
        var b = Run();

        Assert.Equal(a.Select(BitConverter.DoubleToInt64Bits), b.Select(BitConverter.DoubleToInt64Bits));
    }
}
