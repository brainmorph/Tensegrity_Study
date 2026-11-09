using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 4 (PLAN.md section 7): a mass hanging on a cable oscillates at
/// ω = √(k/m) and, with damping, decays at γ = c/(2m), as long as the cable
/// stays taut.
/// </summary>
public class CableOscillatorTests
{
    private const double Dt = 1e-4;

    /// <summary>Runs from rest at y_eq − A and returns the worst deviation from <paramref name="exact"/> in y and ẏ.</summary>
    private static (double Worst, long Events) Run(double damping, double amplitude, double duration, Func<double, (double Y, double Vy)> exact)
    {
        var system = CableOscillator.CreateSystem(damping);
        var theory = CableOscillator.CreateTheory(damping);
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - amplitude, 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var worst = 0.0;

        FixedStepRunner.Run(state, stepper, Dt, (long)Math.Round(duration / Dt), (_, s) =>
        {
            var (y, vy) = exact(s.Time);
            var twist = system.GetTwist(s.Values, 0);
            var pose = system.GetPose(s.Values, 0);
            worst = Math.Max(worst, Math.Max(Math.Abs(pose.Position.Y - y), Math.Abs(twist.Linear.Y - vy)));

            // The force passes through the center of mass: no sideways motion, no turning.
            Assert.Equal(0.0, pose.Position.X);
            Assert.Equal(0.0, pose.Angle);
        });

        return (worst, stepper.EventCount);
    }

    [Fact]
    public void UndampedOscillationIsAtSqrtKOverM()
    {
        var theory = CableOscillator.CreateTheory();
        var omega = Math.Sqrt(CableOscillator.Stiffness / CableOscillator.Mass);
        var amplitude = 0.5 * theory.StaticStretch;

        // About five periods of y = y_eq − A·cos(√(k/m)·t).
        var (worst, events) = Run(0, amplitude, 1.0, t =>
            (theory.EquilibriumY - (amplitude * Math.Cos(omega * t)), amplitude * omega * Math.Sin(omega * t)));

        Assert.Equal(0, events);
        Assert.InRange(worst, 0.0, 1e-11);
    }

    [Fact]
    public void DampedOscillationDecaysAtCOverTwoM()
    {
        const double damping = 8.0; // γ = c/(2m) = 2 s⁻¹, ζ ≈ 0.06.
        var theory = CableOscillator.CreateTheory(damping);
        var amplitude = 0.5 * theory.StaticStretch;
        Assert.Equal(2.0, theory.DecayRate, 15);

        var (worst, events) = Run(damping, amplitude, 1.0, t => theory.Taut(amplitude, t));

        Assert.Equal(0, events);
        Assert.InRange(worst, 0.0, 1e-11);
    }

    [Fact]
    public void SuccessiveTroughsShrinkByTheDecayFactor()
    {
        const double damping = 8.0;
        var system = CableOscillator.CreateSystem(damping);
        var theory = CableOscillator.CreateTheory(damping);
        var amplitude = 0.5 * theory.StaticStretch;
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - amplitude, 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var troughs = new List<double>();
        double? previousVy = null;

        // A trough is where ẏ turns from negative to positive; take u there.
        FixedStepRunner.Run(state, stepper, Dt, 12_000, (_, s) =>
        {
            var vy = system.GetTwist(s.Values, 0).Linear.Y;
            if (previousVy is < 0 && vy >= 0)
            {
                troughs.Add(theory.EquilibriumY - system.GetPose(s.Values, 0).Position.Y);
            }

            previousVy = vy;
        });

        // Over one damped period 2π/ω_d the envelope shrinks by e^(−γ·2π/ω_d).
        // Troughs are sampled to the nearest step, so allow for that.
        var expected = Math.Exp(-theory.DecayRate * 2 * Math.PI / theory.DampedOmega);
        Assert.True(troughs.Count >= 4);
        for (var i = 0; i + 1 < troughs.Count; i++)
        {
            Assert.Equal(expected, troughs[i + 1] / troughs[i], 1e-5);
        }
    }
}
