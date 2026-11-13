using Tensegrity.Core.Analysis;
using Tensegrity.Core.Integration;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>
/// The one-input coupled wing's statics (brainstorm-coupled-wing.md),
/// checked in closed form on the parallelogram and against the simulator:
/// the same energy at any elbow angle, the same servo holding torque along
/// the fold, exactly two rest poses (open and folded) that need no servo
/// torque, and a released rest pose that stays put.
/// </summary>
public sealed class CoupledWingTests
{
    private static readonly CoupledWingGeometry Geometry = new();

    private static readonly CoupledWingStatics Statics = new(Geometry);

    /// <summary>Elbow angles across the fold, between the stops and pressing into each.</summary>
    public static TheoryData<double> ElbowAngles() => [-152.0, -150.0, -135.0, -110.0, -75.0, -40.0, -10.0, 0.0, 2.0];

    [Theory]
    [MemberData(nameof(ElbowAngles))]
    public void ParallelogramKeepsTheHandParallelToTheUpperArm(double elbowDegrees)
    {
        var elbow = Radians(elbowDegrees);
        var pose = Statics.Pose(elbow);

        Assert.Equal(0, pose.Hand, 1e-12);
        Assert.Equal(elbow, pose.Radius, 1e-12);

        // The ulna and radius swing about their roots and the hand only translates,
        // so gravity sees one pendulum: V = g·(m_u·L/2 + m_r·L/2 + m_h·L)·sin φ + const.
        var moment = Geometry.Gravity * ((Geometry.UlnaMass * Geometry.UlnaLength / 2) + (Geometry.RadiusMass * Geometry.RadiusLength / 2) + (Geometry.HandMass * Geometry.UlnaLength));
        Assert.Equal(moment * Math.Sin(elbow), Statics.GravityEnergy(elbow) - Statics.GravityEnergy(0), 1e-12);
    }

    [Theory]
    [MemberData(nameof(ElbowAngles))]
    public void EnergyAndTensionMatchTheSimulator(double elbowDegrees)
    {
        var elbow = Radians(elbowDegrees);
        var model = CoupledWing.Create(Geometry, elbow);
        var state = model.InitialState;

        Assert.True(model.System.PinPositionResidual(state) < 1e-12);
        Assert.Equal(elbow, model.System.GetJointAngle(state, 0).Angle, 1e-12);
        var simulated = model.System.GravitationalPotentialEnergy(state) + model.System.CableElasticEnergy(state) + model.System.LimitElasticEnergy(state);
        Assert.Equal(simulated, Statics.Energy(elbow), 1e-12);
        var tensions = new double[1];
        model.System.GetCableTensions(state, tensions);
        Assert.Equal(tensions[0], Statics.LigamentTension(elbow), 1e-9);
    }

    [Theory]
    [MemberData(nameof(ElbowAngles))]
    public void ServoTorqueMatchesTheSimulator(double elbowDegrees)
    {
        var elbow = Radians(elbowDegrees);
        var model = CoupledWing.Create(Geometry, elbow);

        var held = model.System.SolveHoldingTorques(0, model.InitialState, [CoupledWing.Elbow]);

        Assert.True(held.MaxLinearAcceleration < 1e-7 && held.MaxAngularAcceleration < 1e-6, $"unheld {held.MaxLinearAcceleration:E2} m/s2, {held.MaxAngularAcceleration:E2} rad/s2");
        Assert.Equal(held[CoupledWing.Elbow], Statics.ServoTorque(elbow), 1e-6);
    }

    [Fact]
    public void LigamentPressesBothStopsWithAtLeastThePreload()
    {
        Assert.Equal(Geometry.Preload, Math.Min(Statics.OpenPreload, Statics.FoldPreload), 1e-9);
        Assert.Equal(Geometry.OpenAngle - (Statics.OpenPreload / Geometry.StopStiffness), Statics.OpenStop, 1e-15);
        Assert.Equal(Geometry.FoldAngle + (Statics.FoldPreload / Geometry.StopStiffness), Statics.FoldStop, 1e-15);

        // It pulls the elbow open on one side of the over-center angle and folded on the other.
        Assert.True(Statics.ServoTorque(Geometry.OverCenterAngle + Radians(30)) < 0);
        Assert.True(Statics.ServoTorque(Geometry.OverCenterAngle - Radians(30)) > 0);
    }

    [Fact]
    public void RestsOnlyOpenAndFolded()
    {
        var rests = Statics.RestPoses();

        Assert.Equal(2, rests.Count);
        Assert.Equal(Geometry.FoldAngle, rests[0].Elbow, 1e-7);
        Assert.Equal(Geometry.OpenAngle, rests[1].Elbow, 1e-7);
        Assert.All(rests, rest => Assert.True(rest.Stiffness > 0));
    }

    [Fact]
    public void BothRestPosesHoldWithZeroServoTorqueInTheSimulator()
    {
        foreach (var elbow in new[] { Geometry.OpenAngle, Geometry.FoldAngle })
        {
            var model = CoupledWing.Create(Geometry, elbow);
            var state = model.InitialState;

            var held = model.System.SolveHoldingTorques(0, state, [CoupledWing.Elbow]);
            Assert.True(Math.Abs(held[CoupledWing.Elbow]) < 1e-9, $"servo torque {held[CoupledWing.Elbow]:E2} N m at {elbow}");

            // And with no torque anywhere, nothing accelerates: the lock holds itself.
            var free = model.System.SolveHoldingTorques(0, state, []);
            Assert.True(free.MaxLinearAcceleration < 1e-7 && free.MaxAngularAcceleration < 1e-6, $"unheld {free.MaxLinearAcceleration:E2} m/s2, {free.MaxAngularAcceleration:E2} rad/s2");
        }
    }

    [Fact]
    public void ReleasedAtARestPoseTheSimulatorStaysPut()
    {
        foreach (var elbow in new[] { Geometry.OpenAngle, Geometry.FoldAngle })
        {
            var model = CoupledWing.Create(Geometry, elbow);
            var state = model.CreateInitialState();
            var stepper = new EventSplittingStepper(new Rk4Integrator(), model.System);

            FixedStepRunner.Run(state, stepper, 1e-4, 2_000);

            Assert.Equal(elbow, model.System.GetJointAngle(state.Values, 0).Angle, 1e-6);
        }
    }

    [Fact]
    public void FourBarCannotCloseBeyondItsReach()
    {
        var stretched = Geometry with { RadiusLength = 0.30 };

        Assert.Throws<ArgumentOutOfRangeException>(() => new CoupledWingStatics(stretched).Pose(Radians(15)));
    }

    private static double Radians(double degrees) => degrees * Math.PI / 180;
}
