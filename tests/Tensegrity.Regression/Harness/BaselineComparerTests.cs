namespace Tensegrity.Regression.Harness;

public class BaselineComparerTests
{
    private static GoldenBaseline Baseline(Tolerance tolerance, params double[] values) =>
        new("test", new Dictionary<string, double[]> { ["q"] = values }, new Dictionary<string, Tolerance> { ["q"] = tolerance });

    private static Dictionary<string, double[]> Output(params double[] values) => new() { ["q"] = values };

    [Fact]
    public void IdenticalOutputPasses()
    {
        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 1.0, -2.5, 0.0), Output(1.0, -2.5, 0.0));

        Assert.True(report.Passed, report.ToString());
    }

    [Fact]
    public void ExactToleranceRejectsOneUlpDifference()
    {
        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 1.0), Output(Math.BitIncrement(1.0)));

        Assert.False(report.Passed);
    }

    [Fact]
    public void ExactToleranceTreatsSignedZerosAsEqual()
    {
        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 0.0), Output(-0.0));

        Assert.True(report.Passed, report.ToString());
    }

    [Theory]
    [InlineData(0.9e-6, true)]
    [InlineData(-0.9e-6, true)]
    [InlineData(1.1e-6, false)]
    [InlineData(-1.1e-6, false)]
    public void AbsoluteToleranceBoundsTheDifference(double offset, bool passes)
    {
        var report = BaselineComparer.Compare(Baseline(new Tolerance(1e-6, 0), 5.0), Output(5.0 + offset));

        Assert.Equal(passes, report.Passed);
    }

    [Fact]
    public void ToleranceBoundaryIsInclusive()
    {
        // Exactly representable values, so the difference is exactly the bound.
        var tolerance = new Tolerance(0.5, 0);

        Assert.True(BaselineComparer.Compare(Baseline(tolerance, 2.0), Output(2.5)).Passed);
        Assert.False(BaselineComparer.Compare(Baseline(tolerance, 2.0), Output(Math.BitIncrement(2.5))).Passed);
    }

    [Fact]
    public void RelativeToleranceScalesWithExpectedMagnitude()
    {
        var tolerance = new Tolerance(0, 1e-3);

        // Same absolute difference scale, different verdicts: 0.9 ≤ 1.0 allowed at 1000, but 0.0011 > 0.001 at 1.
        Assert.True(BaselineComparer.Compare(Baseline(tolerance, 1000.0), Output(1000.9)).Passed);
        Assert.False(BaselineComparer.Compare(Baseline(tolerance, 1.0), Output(1.0011)).Passed);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonFiniteOutputFailsEvenWithLooseTolerance(double bad)
    {
        var report = BaselineComparer.Compare(Baseline(new Tolerance(1e9, 1e9), 1.0), Output(bad));

        Assert.False(report.Passed);
    }

    [Fact]
    public void LengthMismatchFails()
    {
        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 1.0, 2.0), Output(1.0));

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("length 1", StringComparison.Ordinal));
    }

    [Fact]
    public void MissingQuantityFails()
    {
        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 1.0), new Dictionary<string, double[]>());

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("was not produced", StringComparison.Ordinal));
    }

    [Fact]
    public void ExtraQuantityFails()
    {
        var output = Output(1.0);
        output["new"] = [2.0];

        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, 1.0), output);

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("'new' is produced but not in the baseline", StringComparison.Ordinal));
    }

    [Fact]
    public void QuantityWithoutToleranceFails()
    {
        var baseline = new GoldenBaseline(
            "test",
            new Dictionary<string, double[]> { ["q"] = [1.0] },
            new Dictionary<string, Tolerance>());

        var report = BaselineComparer.Compare(baseline, Output(1.0));

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("has no tolerance", StringComparison.Ordinal));
    }

    [Fact]
    public void StaleToleranceFails()
    {
        var baseline = new GoldenBaseline(
            "test",
            new Dictionary<string, double[]> { ["q"] = [1.0] },
            new Dictionary<string, Tolerance> { ["q"] = Tolerance.Exact, ["gone"] = Tolerance.Exact });

        var report = BaselineComparer.Compare(baseline, Output(1.0));

        Assert.False(report.Passed);
        Assert.Contains(report.Failures, f => f.Contains("stale tolerance", StringComparison.Ordinal));
    }

    [Fact]
    public void ReportTruncatesManyFailuresPerQuantity()
    {
        var expected = new double[20];
        var actual = Enumerable.Repeat(1.0, 20).ToArray();

        var report = BaselineComparer.Compare(Baseline(Tolerance.Exact, expected), Output(actual));

        Assert.Equal(BaselineComparer.MaxReportedValuesPerQuantity + 1, report.Failures.Count);
        Assert.Contains("15 more values", report.Failures[^1], StringComparison.Ordinal);
    }

    [Theory]
    [InlineData(-1e-9, 0)]
    [InlineData(0, -1e-9)]
    [InlineData(double.NaN, 0)]
    [InlineData(0, double.PositiveInfinity)]
    public void ToleranceRejectsNegativeOrNonFiniteBounds(double absolute, double relative)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new Tolerance(absolute, relative));
    }
}
