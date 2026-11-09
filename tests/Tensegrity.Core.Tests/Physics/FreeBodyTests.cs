using Tensegrity.Core.Bodies;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>Test 2 (PLAN.md section 7): free link under gravity, and a free spinning link.</summary>
public class FreeBodyTests
{
    private static SystemState Simulate(PlanarSystem system, Pose2d pose, Twist2d twist, double dt, long steps)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, pose, twist);
        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, dt, steps);
        return state;
    }

    [Fact]
    public void FreeFallFollowsBallisticTrajectory()
    {
        var system = FreeFallScenario.CreateSystem();
        var p0 = FreeFallScenario.InitialPose;
        var v0 = FreeFallScenario.InitialTwist.Linear;
        var g = -PlanarSystem.StandardGravity.Y;
        var recorded = new List<(double T, Pose2d Pose)>();

        var state = system.CreateState();
        system.SetBody(state.Values, 0, p0, FreeFallScenario.InitialTwist);
        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, FreeFallScenario.Dt, FreeFallScenario.Steps,
            (_, s) => recorded.Add((s.Time, system.GetPose(s.Values, 0))), stride: 100);

        Assert.Equal(21, recorded.Count);
        foreach (var (t, pose) in recorded)
        {
            // RK4 integrates a quadratic trajectory exactly, so only rounding remains.
            Assert.Equal(p0.Position.X + (v0.X * t), pose.Position.X, 1e-12);
            Assert.Equal(p0.Position.Y + (v0.Y * t) - (0.5 * g * t * t), pose.Position.Y, 1e-12);
            Assert.Equal(p0.Angle, pose.Angle);
        }
    }

    [Fact]
    public void FreeFallConservesMechanicalEnergy()
    {
        var system = FreeFallScenario.CreateSystem();
        var start = system.CreateState();
        system.SetBody(start.Values, 0, FreeFallScenario.InitialPose, FreeFallScenario.InitialTwist);
        var e0 = system.KineticEnergy(start.Values) + system.GravitationalPotentialEnergy(start.Values);

        var end = Simulate(system, FreeFallScenario.InitialPose, FreeFallScenario.InitialTwist, FreeFallScenario.Dt, FreeFallScenario.Steps);
        var e1 = system.KineticEnergy(end.Values) + system.GravitationalPotentialEnergy(end.Values);

        Assert.Equal(e0, e1, 1e-10);
    }

    [Fact]
    public void SpinningBodyRotatesUniformlyAndConservesEnergyAndMomentum()
    {
        var system = SpinningBodyScenario.CreateSystem();
        var pose0 = SpinningBodyScenario.InitialPose;
        var twist0 = SpinningBodyScenario.InitialTwist;
        var start = system.CreateState();
        system.SetBody(start.Values, 0, pose0, twist0);
        var ke0 = system.KineticEnergy(start.Values);
        var l0 = system.AngularMomentum(start.Values, Vector2d.Zero);

        var end = Simulate(system, pose0, twist0, SpinningBodyScenario.Dt, SpinningBodyScenario.Steps);
        var t = end.Time;
        var pose = system.GetPose(end.Values, 0);

        Assert.Equal(pose0.Angle + (twist0.Angular * t), pose.Angle, 1e-12);
        Assert.Equal(pose0.Position.X + (twist0.Linear.X * t), pose.Position.X, 1e-12);
        Assert.Equal(pose0.Position.Y + (twist0.Linear.Y * t), pose.Position.Y, 1e-12);
        Assert.Equal(ke0, system.KineticEnergy(end.Values), 1e-13);
        Assert.Equal(l0, system.AngularMomentum(end.Values, Vector2d.Zero), 1e-12);
    }

    [Fact]
    public void BodiesInOneSystemEvolveIndependently()
    {
        // Two bodies with no coupling must each match a run of that body alone, bit for bit.
        var a = PlanarLink.SlenderBar("a", 2.0, 0.5);
        var b = PlanarLink.SlenderBar("b", 0.7, 1.3);
        var poseA = new Pose2d(new Vector2d(0, 5), 0.1);
        var twistA = new Twist2d(new Vector2d(1, 2), -0.5);
        var poseB = new Pose2d(new Vector2d(-3, 1), 2.0);
        var twistB = new Twist2d(new Vector2d(-0.2, 0.0), 4.0);
        var g = PlanarSystem.StandardGravity;

        var both = new PlanarSystem([a, b], g);
        var state = both.CreateState();
        both.SetBody(state.Values, 0, poseA, twistA);
        both.SetBody(state.Values, 1, poseB, twistB);
        FixedStepRunner.Run(state, new Rk4Integrator(), both.Evaluate, 1e-3, 500);

        var aloneA = Simulate(new PlanarSystem([a], g), poseA, twistA, 1e-3, 500);
        var aloneB = Simulate(new PlanarSystem([b], g), poseB, twistB, 1e-3, 500);

        Assert.Equal(new PlanarSystem([a], g).GetPose(aloneA.Values, 0), both.GetPose(state.Values, 0));
        Assert.Equal(new PlanarSystem([b], g).GetPose(aloneB.Values, 0), both.GetPose(state.Values, 1));
        Assert.Equal(new PlanarSystem([b], g).GetTwist(aloneB.Values, 0), both.GetTwist(state.Values, 1));
    }
}
