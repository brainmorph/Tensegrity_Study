using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 6 (PLAN.md section 7): pin constraints hold over long runs, and drift
/// is controlled exactly as designed by Baumgarte stabilization.
/// </summary>
public class PinConstraintTests
{
    private static readonly PlanarLink Upper = PlanarLink.SlenderBar("upper", mass: 1.0, length: 1.0);
    private static readonly PlanarLink Lower = PlanarLink.SlenderBar("lower", mass: 0.6, length: 0.8);

    /// <summary>Two links in a chain: upper pinned to the ground at the origin, lower pinned to upper's far end.</summary>
    private static PlanarSystem DoublePendulum(Vector2d gravity) => new(
        [Upper, Lower],
        gravity,
        [
            new PinJoint("shoulder", Attachment.OnLink("upper", Upper.EndA), Attachment.Ground(Vector2d.Zero)),
            new PinJoint("elbow", Attachment.OnLink("lower", Lower.EndA), Attachment.OnLink("upper", Upper.EndB)),
        ]);

    /// <summary>Writes a double-pendulum state with both pins satisfied exactly.</summary>
    private static SystemState DoublePendulumState(PlanarSystem system, double theta1, double omega1, double theta2, double omega2)
    {
        var state = system.CreateState();
        var r1 = (-Upper.EndA).Rotated(theta1);
        var elbow = Upper.EndB.Rotated(theta1) + r1;
        var elbowVelocity = Vector2d.Cross(omega1, elbow);
        var r2 = (-Lower.EndA).Rotated(theta2);
        system.SetBody(state.Values, 0, new Pose2d(r1, theta1), new Twist2d(Vector2d.Cross(omega1, r1), omega1));
        system.SetBody(state.Values, 1, new Pose2d(elbow + r2, theta2), new Twist2d(elbowVelocity + Vector2d.Cross(omega2, r2), omega2));
        return state;
    }

    [Fact]
    public void PendulumPinResidualStaysTinyAndDoesNotGrowOverALongRun()
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle + 1.0, 0.0);
        const long steps = 200_000;
        const long window = steps / 10;
        var firstPosition = 0.0;
        var firstVelocity = 0.0;
        var lastPosition = 0.0;
        var lastVelocity = 0.0;

        // 20 s ≈ 12 periods, compared between the first and last 2 s windows.
        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, steps, (k, s) =>
        {
            var position = system.PinPositionResidual(s.Values);
            var velocity = system.PinVelocityResidual(s.Values);
            if (k < window)
            {
                firstPosition = Math.Max(firstPosition, position);
                firstVelocity = Math.Max(firstVelocity, velocity);
            }
            else if (k >= steps - window)
            {
                lastPosition = Math.Max(lastPosition, position);
                lastVelocity = Math.Max(lastVelocity, velocity);
            }
        });

        // Measured: ≈ 2e-14 m and ≈ 1.8e-12 m/s (peak speed ≈ 2.6 m/s),
        // the same in every window. Stabilization holds the error at the
        // per-step truncation level instead of letting it accumulate.
        Assert.InRange(lastPosition, 0.0, 1e-13);
        Assert.InRange(lastVelocity, 0.0, 1e-11);
        Assert.InRange(lastPosition, 0.0, 2 * firstPosition);
        Assert.InRange(lastVelocity, 0.0, 2 * firstVelocity);
    }

    [Fact]
    public void DoublePendulumHoldsBothPinsAndConservesEnergy()
    {
        var system = DoublePendulum(PlanarSystem.StandardGravity);
        var state = DoublePendulumState(system, theta1: 0.3, omega1: 0.0, theta2: 1.2, omega2: 2.0);
        double Energy(double[] y) => system.KineticEnergy(y) + system.GravitationalPotentialEnergy(y);
        var initialEnergy = Energy(state.Values);
        var maxResidual = 0.0;
        var maxDrift = 0.0;

        // Chaotic regime at this energy: a strong test of the constraint solve.
        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, 50_000, (_, s) =>
        {
            maxResidual = Math.Max(maxResidual, system.PinPositionResidual(s.Values));
            maxDrift = Math.Max(maxDrift, Math.Abs(Energy(s.Values) - initialEnergy));
        });

        Assert.InRange(maxResidual, 0.0, 1e-12);
        Assert.InRange(maxDrift, 0.0, 1e-9);
    }

    [Fact]
    public void HangingChainPinsCarryTheWeightBelowThem()
    {
        var system = DoublePendulum(PlanarSystem.StandardGravity);
        var state = DoublePendulumState(system, -Math.PI / 2, 0, -Math.PI / 2, 0);
        var reactions = new Vector2d[2];
        var dy = new double[system.StateLength];

        system.Evaluate(0, state.Values, dy);
        system.GetPinReactions(state.Values, reactions);

        // Not exactly zero: −π/2 is not representable (see PendulumTests).
        Assert.All(dy, d => Assert.Equal(0.0, d, 1e-12));
        Assert.Equal((Upper.Mass + Lower.Mass) * 9.81, reactions[0].Y, 1e-12);
        Assert.Equal(Lower.Mass * 9.81, reactions[1].Y, 1e-12);
        Assert.Equal(0.0, reactions[0].X, 1e-12);
        Assert.Equal(0.0, reactions[1].X, 1e-12);
    }

    /// <summary>
    /// With stabilization, the constraint error obeys C̈ + 2ω·Ċ + ω²·C = 0
    /// exactly, so a violated pin released at rest returns as
    /// C(t) = C₀·(1 + ω·t)·e^(−ω·t). This checks the drift control is the
    /// designed critically damped response, not just "small".
    /// </summary>
    [Fact]
    public void StabilizationRemovesViolationAsCriticallyDampedMode()
    {
        var stabilization = new ConstraintStabilization(40);
        var system = PendulumScenario.CreateSystem(stabilization);
        var theory = PendulumScenario.CreateTheory();
        var state = PendulumScenario.CreateState(system, theory, 0.7, 0.0);
        var offset = new Vector2d(2e-3, -1e-3);
        var pose = system.GetPose(state.Values, 0);
        system.SetBody(state.Values, 0, pose with { Position = pose.Position + offset }, default);
        var link = PendulumScenario.CreateLink();
        var attachment = Attachment.OnLink(link.Id, link.EndA);
        var omega = stabilization.Frequency;
        var worst = 0.0;

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, 5_000, (_, s) =>
        {
            var c = system.WorldPoint(s.Values, attachment);
            var expected = offset * ((1 + (omega * s.Time)) * Math.Exp(-omega * s.Time));
            worst = Math.Max(worst, (c - expected).Length);
        }, stride: 100);

        Assert.InRange(worst, 0.0, 1e-12);
        Assert.InRange(system.PinPositionResidual(state.Values), 0.0, offset.Length * 1e-6);
    }

    [Fact]
    public void WithoutStabilizationAViolationIsNeitherCorrectedNorGrown()
    {
        var system = PendulumScenario.CreateSystem(ConstraintStabilization.None);
        var theory = PendulumScenario.CreateTheory();
        var state = PendulumScenario.CreateState(system, theory, 0.7, 0.0);
        var offset = new Vector2d(2e-3, -1e-3);
        var pose = system.GetPose(state.Values, 0);
        system.SetBody(state.Values, 0, pose with { Position = pose.Position + offset }, default);

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, 1e-4, 5_000);

        // C̈ = 0 with Ċ(0) = 0: the error is frozen at its initial value.
        Assert.Equal(offset.Length, system.PinPositionResidual(state.Values), 1e-12);
    }

    [Fact]
    public void RedundantPinsAreReportedAsSingular()
    {
        var link = PendulumScenario.CreateLink();
        PinJoint Pin(string id) => new(id, Attachment.OnLink(link.Id, link.EndA), Attachment.Ground(Vector2d.Zero));
        var system = new PlanarSystem([link], PlanarSystem.StandardGravity, [Pin("p1"), Pin("p2")]);
        var state = system.CreateState();

        var error = Assert.Throws<InvalidOperationException>(
            () => system.Evaluate(0, state.Values, new double[system.StateLength]));
        Assert.Contains("singular", error.Message, StringComparison.Ordinal);
    }
}
