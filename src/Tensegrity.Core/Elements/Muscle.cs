namespace Tensegrity.Core.Elements;

/// <summary>
/// The contractile part of a series-elastic muscle (PLAN.md M9). It sits in
/// series with a <see cref="Cable"/>, which is the muscle's elastic tendon:
/// the path between the cable's ends is the contractile length ℓ plus the
/// tendon's length, so the tendon's stretch is L − ℓ − L0.
///
/// While active, the contractile part shortens at a constant speed, scaled
/// by its activation a(t) ∈ [0, 1]:
/// <code>
/// dℓ/dt = −v·a(t)
/// </code>
/// whatever the load. As it shortens, it stretches the tendon, so the
/// tension rises slowly and is carried by the tendon's spring. The
/// activation is a piecewise-linear <see cref="Schedule"/>, latched by
/// segment like an actuator's torque (PLAN.md 4.2), so ℓ is a smooth
/// quadratic inside each segment.
///
/// The activation must end at zero, so the muscle shortens by a finite
/// amount, and that amount must not exceed the initial contractile length.
/// The muscle does work T·v·a on the system, counted with the actuators'.
/// </summary>
public sealed class Muscle
{
    public Muscle(double initialLength, double shorteningSpeed, Schedule activation)
    {
        RequirePositive(initialLength, nameof(initialLength));
        RequirePositive(shorteningSpeed, nameof(shorteningSpeed));
        ArgumentNullException.ThrowIfNull(activation);

        foreach (var point in activation.Points)
        {
            if (point.Time < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(activation), point, "Activation times must not be negative.");
            }

            if (point.Value < 0 || point.Value > 1)
            {
                throw new ArgumentOutOfRangeException(nameof(activation), point, "Activation must lie in [0, 1].");
            }
        }

        if (activation.Points[^1].Value != 0)
        {
            throw new ArgumentException("Activation must end at zero, so the muscle shortens by a finite amount.", nameof(activation));
        }

        InitialLength = initialLength;
        ShorteningSpeed = shorteningSpeed;
        Activation = activation;
        TotalShortening = shorteningSpeed * ActivationIntegral(double.PositiveInfinity);
        if (TotalShortening > initialLength)
        {
            throw new ArgumentException(
                $"The muscle would shorten by {TotalShortening} m, more than its initial contractile length of {initialLength} m.",
                nameof(activation));
        }
    }

    /// <summary>Contractile length ℓ at t = 0, m.</summary>
    public double InitialLength { get; }

    /// <summary>Shortening speed v at full activation, m/s.</summary>
    public double ShorteningSpeed { get; }

    /// <summary>Activation a(t), from 0 (relaxed) to 1 (full speed).</summary>
    public Schedule Activation { get; }

    /// <summary>How far the contractile part shortens over the whole run, v·∫a dt, m.</summary>
    public double TotalShortening { get; }

    /// <summary>Contractile rate dℓ/dt = −v·a(t) at <paramref name="time"/>, m/s.</summary>
    public double RateAt(double time) => -ShorteningSpeed * Activation.ValueAt(time);

    /// <summary>The rate from activation segment <paramref name="segment"/>'s formula, used while the segment is latched.</summary>
    public double RateInSegment(int segment, double time) => -ShorteningSpeed * Activation.ValueInSegment(segment, time);

    /// <summary>Contractile length ℓ(t) in closed form, m. The simulator integrates ℓ as a state; this is its check.</summary>
    public double LengthAt(double time) => InitialLength - (ShorteningSpeed * ActivationIntegral(time));

    /// <summary>∫ a dt from 0 to <paramref name="end"/>: exact for the piecewise-linear schedule.</summary>
    private double ActivationIntegral(double end)
    {
        if (!(end > 0))
        {
            return 0.0;
        }

        var points = Activation.Points;
        var first = points[0];
        var total = first.Value * Math.Min(end, first.Time);
        for (var i = 1; i < points.Count; i++)
        {
            var a = points[i - 1];
            var b = points[i];
            if (end <= a.Time)
            {
                return total;
            }

            var t = Math.Min(end, b.Time);
            var value = a.Value + ((b.Value - a.Value) * (t - a.Time) / (b.Time - a.Time));
            total += 0.5 * (a.Value + value) * (t - a.Time);
        }

        // After the last breakpoint the activation is zero.
        return total;
    }

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }
}
