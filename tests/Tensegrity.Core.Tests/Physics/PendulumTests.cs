using Tensegrity.Core.Integration;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 3 (PLAN.md section 7): link pinned at one end. Uses the reduced
/// one-angle model until M2 adds pin constraints (see <see cref="ReducedPendulum"/>).
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
    private static double MeasurePeriod(ReducedPendulum pendulum, double amplitude, double dt, int periods)
    {
        var hang = pendulum.HangingAngle;
        var estimate = pendulum.ExactPeriod(amplitude);
        var steps = (long)Math.Ceiling(((periods + 0.5) * estimate) / dt);
        var state = new SystemState([hang + amplitude, 0.0]);
        var crossings = new List<double>();
        (double T, double Phi, double Omega)? previous = null;

        FixedStepRunner.Run(state, new Rk4Integrator(), pendulum.Evaluate, dt, steps, (_, s) =>
        {
            var current = (s.Time, s.Values[0] - hang, s.Values[1]);
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
        Assert.Equal(Math.PI / 2, ReducedPendulum.CompleteEllipticK(0), 15);

        // K(k² = 1/2) = Γ(1/4)² / (4√π) = 1.854074677301372 (A&S 17.3).
        Assert.Equal(1.854074677301372, ReducedPendulum.CompleteEllipticK(Math.Sqrt(0.5)), 14);
    }

    [Fact]
    public void PivotInertiaAndPeriodFormulaForUniformBar()
    {
        var pendulum = PendulumScenario.CreatePendulum();

        // Uniform bar about its end: I = m·L²/3 = 1/3; d = L/2.
        Assert.Equal(1.0 / 3.0, pendulum.PivotInertia, 15);
        Assert.Equal(0.5, pendulum.Distance, 15);
        Assert.Equal(2 * Math.PI * Math.Sqrt(2.0 / (3.0 * 9.81)), pendulum.SmallAmplitudePeriod, 14);
        Assert.Equal(-Math.PI / 2, pendulum.HangingAngle, 15);
    }

    [Fact]
    public void HangingAtRestIsAnEquilibrium()
    {
        var pendulum = PendulumScenario.CreatePendulum();
        Span<double> dy = stackalloc double[2];

        pendulum.Evaluate(0, [pendulum.HangingAngle, 0.0], dy);

        // Not exactly zero: −π/2 is not representable, so cos(θ) ≈ 6.1e-17 and
        // θ̈ ≈ 6.1e-17 · m·g·d / I_pivot ≈ 9e-16 rad/s².
        Assert.Equal(0.0, dy[0]);
        Assert.Equal(0.0, dy[1], 1e-14);
    }

    [Fact]
    public void SmallAmplitudePeriodMatchesClosedForm()
    {
        var pendulum = PendulumScenario.CreatePendulum();

        var period = MeasurePeriod(pendulum, amplitude: 1e-3, dt: 1e-4, periods: 5);

        // Finite-amplitude correction at 1e-3 rad is ≈ θ₀²/16 ≈ 6e-8 relative.
        Assert.Equal(pendulum.SmallAmplitudePeriod, period, pendulum.SmallAmplitudePeriod * 1e-6);
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(1.0)]
    [InlineData(2.5)]
    public void LargeAmplitudePeriodMatchesEllipticIntegral(double amplitude)
    {
        var pendulum = PendulumScenario.CreatePendulum();

        var period = MeasurePeriod(pendulum, amplitude, dt: 1e-4, periods: 3);

        Assert.Equal(pendulum.ExactPeriod(amplitude), period, pendulum.ExactPeriod(amplitude) * 1e-9);
    }

    [Fact]
    public void ReconstructedFullStateKeepsPinPointFixed()
    {
        var pendulum = PendulumScenario.CreatePendulum();
        var system = new PlanarSystem([pendulum.Link], pendulum.Gravity);
        var full = new double[system.StateLength];
        const double omega = 1.7;

        pendulum.WriteFullState(system, [0.4, omega], full);
        var pose = system.GetPose(full, 0);
        var twist = system.GetTwist(full, 0);
        var pinWorld = pose.ToWorld(pendulum.Link.EndA);
        var pinVelocity = twist.VelocityOfPoint(pose.RotateToWorld(pendulum.Link.EndA));

        Assert.Equal(0.0, pinWorld.Length, 15);
        Assert.Equal(0.0, pinVelocity.Length, 15);
        Assert.Equal(0.5 * pendulum.PivotInertia * omega * omega, system.KineticEnergy(full), 14);
    }

    [Fact]
    public void EnergyDriftIsSmall()
    {
        var output = new PendulumScenario().Run();

        Assert.InRange(output["max_energy_drift"][0], 0.0, 1e-11);
    }

    /// <summary>
    /// Test 5 on the nonlinear pendulum, where no closed-form trajectory is
    /// used: successive differences under dt halving shrink by ≈ 16.
    /// </summary>
    [Fact]
    public void PendulumTrajectoryConvergesAtFourthOrder()
    {
        var pendulum = PendulumScenario.CreatePendulum();
        double ThetaAt(double dt)
        {
            var state = new SystemState([pendulum.HangingAngle + 1.0, 0.0]);
            FixedStepRunner.Run(state, new Rk4Integrator(), pendulum.Evaluate, dt, (long)Math.Round(2.0 / dt));
            return state.Values[0];
        }

        double[] theta = [ThetaAt(0.02), ThetaAt(0.01), ThetaAt(0.005), ThetaAt(0.0025)];

        for (var i = 0; i + 2 < theta.Length; i++)
        {
            var ratio = (theta[i] - theta[i + 1]) / (theta[i + 1] - theta[i + 2]);
            Assert.InRange(ratio, 14.0, 18.0);
        }
    }
}
