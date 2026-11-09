namespace Tensegrity.Core.Systems;

/// <summary>
/// Baumgarte stabilization for pin constraints (PLAN.md 4.2). Instead of
/// C̈ = 0, the constraint solve enforces
/// <code>
/// C̈ + 2·ω·Ċ + ω²·C = 0
/// </code>
/// so any drift in position (C) or velocity (Ċ) decays as a critically
/// damped oscillator with natural frequency ω, rad/s. On the constraint
/// manifold (C = Ċ = 0) the extra terms vanish and the physics is unchanged.
/// ω = 0 turns stabilization off.
///
/// Keep ω·dt well below 1 so the correction itself is resolved by the
/// integrator.
/// </summary>
public readonly record struct ConstraintStabilization
{
    public ConstraintStabilization(double frequency)
    {
        if (!double.IsFinite(frequency) || frequency < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(frequency), frequency, "Must be finite and non-negative.");
        }

        Frequency = frequency;
    }

    /// <summary>Natural frequency ω of the drift correction, rad/s.</summary>
    public double Frequency { get; }

    /// <summary>No stabilization: the solve enforces C̈ = 0 only.</summary>
    public static ConstraintStabilization None => new(0);

    /// <summary>
    /// ω = 50 rad/s: drift decays with a 20 ms time constant, while
    /// ω·dt = 0.005 at the study's working step of 1e-4 s.
    /// </summary>
    public static ConstraintStabilization Default => new(50);
}
