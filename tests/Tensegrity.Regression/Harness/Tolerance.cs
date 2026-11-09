namespace Tensegrity.Regression.Harness;

/// <summary>
/// Per-quantity comparison tolerance. A value passes when
/// <c>|actual − expected| ≤ Absolute + Relative·|expected|</c>.
/// Both zero means exact equality (0.0 and −0.0 compare equal).
/// Non-finite values (NaN, ±∞) never pass: a baseline must not contain them,
/// and a simulation producing them has failed.
/// </summary>
public readonly record struct Tolerance
{
    public Tolerance(double absolute, double relative)
    {
        if (!double.IsFinite(absolute) || absolute < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(absolute), absolute, "Must be finite and non-negative.");
        }

        if (!double.IsFinite(relative) || relative < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(relative), relative, "Must be finite and non-negative.");
        }

        Absolute = absolute;
        Relative = relative;
    }

    public double Absolute { get; }

    public double Relative { get; }

    public static Tolerance Exact => new(0, 0);

    /// <summary>The largest difference allowed for a given expected value.</summary>
    public double AllowedDifference(double expected) => Absolute + (Relative * Math.Abs(expected));

    public bool Accepts(double expected, double actual) =>
        double.IsFinite(expected)
        && double.IsFinite(actual)
        && Math.Abs(actual - expected) <= AllowedDifference(expected);

    public override string ToString() => $"abs {Absolute:G6}, rel {Relative:G6}";
}
