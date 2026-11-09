using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 7 (PLAN.md section 7): with no damping, total mechanical energy is
/// conserved to integrator accuracy and its drift scales as dt⁴; with
/// damping, mechanical energy plus the integrated dissipation is conserved.
/// </summary>
public class EnergyTests
{
    private static double PendulumDrift(double dt)
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle + 2.5, 0.0);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;
        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, dt, (long)Math.Round(4.0 / dt), (_, s) =>
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));
        return drift;
    }

    /// <summary>Runs the hanging cable from rest at y_eq − A and reports the worst |E(t) − E(0)| of the total energy.</summary>
    private static (double Drift, EventSplittingStepper Stepper, HashSet<CableMode> Modes, double Dissipated, bool Monotone) CableRun(
        double damping, double amplitudeOverStretch, double dt, double duration)
    {
        var system = CableOscillator.CreateSystem(damping);
        var theory = CableOscillator.CreateTheory(damping);
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - (amplitudeOverStretch * theory.StaticStretch), 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;
        var modes = new HashSet<CableMode>();
        var previousDissipated = 0.0;
        var monotone = true;

        FixedStepRunner.Run(state, stepper, dt, (long)Math.Round(duration / dt), (_, s) =>
        {
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial));
            modes.Add(system.GetCableMode(s.Values, 0));
            var dissipated = system.DissipatedEnergy(s.Values);
            monotone &= dissipated >= previousDissipated;
            previousDissipated = dissipated;
        });

        return (drift, stepper, modes, system.DissipatedEnergy(state.Values), monotone);
    }

    [Fact]
    public void PendulumEnergyDriftScalesAsDtToTheFourth()
    {
        // 2.5 rad: strongly nonlinear, where RK4's energy error is O(dt⁴).
        double[] drift = [PendulumDrift(0.005), PendulumDrift(0.0025), PendulumDrift(0.00125)];

        for (var i = 0; i + 1 < drift.Length; i++)
        {
            Assert.InRange(drift[i] / drift[i + 1], 13.0, 19.0);
        }
    }

    [Fact]
    public void LinearCableOscillatorDriftIsAtLeastFourthOrder()
    {
        // For a linear oscillator RK4's energy error is one order better
        // (|R(iωh)|² = 1 − (ωh)⁶/72 + …), so ratios near 32 are expected.
        double[] drift = [CableRun(0, 0.5, 1e-3, 2).Drift, CableRun(0, 0.5, 5e-4, 2).Drift, CableRun(0, 0.5, 2.5e-4, 2).Drift];

        for (var i = 0; i + 1 < drift.Length; i++)
        {
            Assert.True(drift[i] / drift[i + 1] >= 14, $"Drift fell only {drift[i] / drift[i + 1]}x per halving.");
        }
    }

    [Fact]
    public void UndampedBouncingConservesEnergyAcrossSlackAndTautEvents()
    {
        var run = CableRun(0, 3, 1e-4, 2);

        Assert.True(run.Stepper.EventCount >= 10);
        Assert.Equal([CableMode.Slack, CableMode.Taut], run.Modes.Order());
        Assert.Equal(0.0, run.Dissipated);

        // Measured ≈ 2.4e-13 J against a 19.7 J energy scale.
        Assert.InRange(run.Drift, 0.0, 1e-11);
    }

    [Fact]
    public void DampedEnergyBalanceClosesThroughEveryCableMode()
    {
        var run = CableRun(8, 3, 1e-4, 2);

        Assert.Equal([CableMode.Slack, CableMode.Taut, CableMode.Recoiling], run.Modes.Order());
        Assert.True(run.Monotone, "Dissipated energy must never decrease.");
        Assert.True(run.Dissipated > 0.5, $"Only {run.Dissipated} J dissipated; the test needs real damping.");

        // Measured ≈ 1.7e-12 J while ≈ 0.87 J is dissipated.
        Assert.InRange(run.Drift, 0.0, 1e-10);
    }

    [Fact]
    public void DampedEnergyBalanceErrorConvergesAtFourthOrder()
    {
        double[] drift = [CableRun(8, 3, 4e-4, 2).Drift, CableRun(8, 3, 2e-4, 2).Drift, CableRun(8, 3, 1e-4, 2).Drift];

        for (var i = 0; i + 1 < drift.Length; i++)
        {
            Assert.InRange(drift[i] / drift[i + 1], 13.0, 19.0);
        }
    }

    [Fact]
    public void ElasticEnergyIsHalfKStretchSquaredAndZeroWhenSlack()
    {
        var system = CableOscillator.CreateSystem();
        var stretched = CableOscillator.CreateState(system, -CableOscillator.RestLength - 0.01, 0.0);
        var slack = CableOscillator.CreateState(system, -CableOscillator.RestLength + 0.01, 0.0);

        Assert.Equal(0.5 * CableOscillator.Stiffness * 0.01 * 0.01, system.CableElasticEnergy(stretched.Values), 12);
        Assert.Equal(0.0, system.CableElasticEnergy(slack.Values));
    }
}
