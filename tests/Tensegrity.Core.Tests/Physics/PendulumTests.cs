using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 3 (PLAN.md section 7): a link pinned at one end, simulated in maximal
/// coordinates with a Lagrange-multiplier pin and checked against the
/// closed-form period.
/// </summary>
public class PendulumTests
{
    /// <summary>
    /// Integrates from rest at <paramref name="amplitude"/> and returns the
    /// mean period over <paramref name="periods"/> full swings, from the
    /// zero crossings of the swing angle. Each crossing is located on the
    /// cubic Hermite interpolant of the step (uses θ and θ̇ at both ends),
    /// so the timing error is O(dt⁴) like the integrator itself.
    /// </summary>
    private static double MeasurePeriod(double amplitude, double dt, int periods)
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var hang = theory.HangingAngle;
        var estimate = theory.ExactPeriod(amplitude);
        var steps = (long)Math.Ceiling(((periods + 0.5) * estimate) / dt);
        var state = PendulumScenario.CreateState(system, theory, hang + amplitude, 0.0);
        var crossings = new List<double>();
        (double T, double Phi, double Omega)? previous = null;

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, dt, steps, (_, s) =>
        {
            var current = (s.Time, system.GetPose(s.Values, 0).Angle - hang, system.GetTwist(s.Values, 0).Angular);
            if (previous is { } p && Math.Sign(p.Phi) != Math.Sign(current.Item2) && current.Item2 != 0)
            {
                crossings.Add(HermiteRoot(p, current));
            }

            previous = current;
        });

        Assert.True(crossings.Count >= (2 * periods) + 1, $"Only {crossings.Count} crossings found.");
        return (crossings[2 * periods] - crossings[0]) / periods;
    }

    private static double HermiteRoot((double T, double Phi, double Omega) a, (double T, double Phi, double Omega) b)
    {
        var h = b.T - a.T;
        double Eval(double s) =>
            (((2 * s * s * s) - (3 * s * s) + 1) * a.Phi)
            + (((s * s * s) - (2 * s * s) + s) * h * a.Omega)
            + ((-(2 * s * s * s) + (3 * s * s)) * b.Phi)
            + (((s * s * s) - (s * s)) * h * b.Omega);

        double lo = 0, hi = 1;
        var fLo = Eval(lo);
        for (var i = 0; i < 80; i++)
        {
            var mid = 0.5 * (lo + hi);
            var fMid = Eval(mid);
            if (Math.Sign(fMid) == Math.Sign(fLo))
            {
                (lo, fLo) = (mid, fMid);
            }
            else
            {
                hi = mid;
            }
        }

        return a.T + (0.5 * (lo + hi) * h);
    }

    [Fact]
    public void CompleteEllipticKMatchesKnownValues()
    {
        Assert.Equal(Math.PI / 2, PendulumTheory.CompleteEllipticK(0), 15);

        // K(k² = 1/2) = Γ(1/4)² / (4√π) = 1.854074677301372 (A&S 17.3).
        Assert.Equal(1.854074677301372, PendulumTheory.CompleteEllipticK(Math.Sqrt(0.5)), 14);
    }

    [Fact]
    public void PivotInertiaAndPeriodFormulaForUniformBar()
    {
        var theory = PendulumScenario.CreateTheory();

        // Uniform bar about its end: I = m·L²/3 = 1/3; d = L/2.
        Assert.Equal(1.0 / 3.0, theory.PivotInertia, 15);
        Assert.Equal(0.5, theory.Distance, 15);
        Assert.Equal(2 * Math.PI * Math.Sqrt(2.0 / (3.0 * 9.81)), theory.SmallAmplitudePeriod, 14);
        Assert.Equal(-Math.PI / 2, theory.HangingAngle, 15);
    }

    [Fact]
    public void HangingAtRestIsAnEquilibriumAndThePinCarriesTheWeight()
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle, 0.0);
        var dy = new double[system.StateLength];
        var reaction = new Vector2d[1];

        system.Evaluate(0, state.Values, dy);
        system.GetPinReactions(state.Time, state.Values, reaction);

        // −π/2 is not representable, so cos(θ) ≈ 6.1e-17 and the residual
        // accelerations sit at the 1e-15 level rather than exactly zero.
        Assert.All(dy, d => Assert.Equal(0.0, d, 1e-14));
        Assert.Equal(0.0, reaction[0].X, 1e-14);
        Assert.Equal(1.0 * 9.81, reaction[0].Y, 1e-13);
    }

    [Fact]
    public void PinReactionAtTheBottomOfASwingIncludesCentripetalForce()
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        const double omega = 2.5;
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle, omega);
        var reaction = new Vector2d[1];

        system.GetPinReactions(state.Time, state.Values, reaction);

        // Hanging straight down, gravity has no moment, so θ̈ = 0 and the pin
        // supplies weight plus centripetal force: m·(g + ω²·d), straight up.
        Assert.Equal(0.0, reaction[0].X, 1e-13);
        Assert.Equal(1.0 * (9.81 + (omega * omega * theory.Distance)), reaction[0].Y, 1e-12);
    }

    [Fact]
    public void StateAtSatisfiesThePinExactly()
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        const double omega = 1.7;

        var state = PendulumScenario.CreateState(system, theory, 0.4, omega);

        Assert.Equal(0.0, system.PinPositionResidual(state.Values), 15);
        Assert.Equal(0.0, system.PinVelocityResidual(state.Values), 15);
        Assert.Equal(0.5 * theory.PivotInertia * omega * omega, system.KineticEnergy(state.Values), 14);
    }

    [Fact]
    public void SmallAmplitudePeriodMatchesClosedForm()
    {
        var theory = PendulumScenario.CreateTheory();

        var period = MeasurePeriod(amplitude: 1e-3, dt: 1e-4, periods: 5);

        // Finite-amplitude correction at 1e-3 rad is ≈ θ₀²/16 ≈ 6e-8 relative.
        Assert.Equal(theory.SmallAmplitudePeriod, period, theory.SmallAmplitudePeriod * 1e-6);
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(1.0)]
    [InlineData(2.5)]
    public void LargeAmplitudePeriodMatchesEllipticIntegral(double amplitude)
    {
        var theory = PendulumScenario.CreateTheory();

        var period = MeasurePeriod(amplitude, dt: 1e-4, periods: 3);

        Assert.Equal(theory.ExactPeriod(amplitude), period, theory.ExactPeriod(amplitude) * 1e-9);
    }

    [Fact]
    public void EnergyDriftIsSmall()
    {
        var output = new PendulumScenario().Run();

        Assert.InRange(output["max_energy_drift"][0], 0.0, 1e-11);
    }

    /// <summary>
    /// Test 5 on the nonlinear pendulum, where no closed-form trajectory is
    /// used: successive differences under dt halving shrink by ≈ 16. Run
    /// through the pin constraint, so it also shows the constraint solve
    /// keeps RK4 at fourth order.
    /// </summary>
    [Fact]
    public void PendulumTrajectoryConvergesAtFourthOrder()
    {
        var theory = PendulumScenario.CreateTheory();
        double ThetaAt(double dt)
        {
            var system = PendulumScenario.CreateSystem();
            var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle + 1.0, 0.0);
            FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, dt, (long)Math.Round(2.0 / dt));
            return system.GetPose(state.Values, 0).Angle;
        }

        double[] theta = [ThetaAt(0.02), ThetaAt(0.01), ThetaAt(0.005), ThetaAt(0.0025)];

        for (var i = 0; i + 2 < theta.Length; i++)
        {
            var ratio = (theta[i] - theta[i + 1]) / (theta[i + 1] - theta[i + 2]);
            Assert.InRange(ratio, 14.0, 18.0);
        }
    }
}
