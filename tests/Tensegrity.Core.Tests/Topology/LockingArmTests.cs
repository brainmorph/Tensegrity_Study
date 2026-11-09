using Tensegrity.Core.Integration;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Topology;

/// <summary>The locking-arm generator: its pose is an exact zero-torque equilibrium.</summary>
public class LockingArmTests
{
    private static double[] Tensions(PlanarModel model)
    {
        var tensions = new double[model.System.Cables.Count];
        model.System.GetCableTensions(model.InitialState, tensions);
        return tensions;
    }

    [Theory]
    [InlineData(0.0, 0.0)]
    [InlineData(0.2, -0.3)]
    [InlineData(-0.4, 0.25)]
    public void TheTendonsHoldThePoseWithZeroTorque(double upperAngle, double lowerAngle)
    {
        var model = LockingArm.Create(upperAngle: upperAngle, lowerAngle: lowerAngle);

        var hold = model.System.SolveHoldingTorques(0, model.InitialState, [LockingArm.Shoulder, LockingArm.Elbow]);

        Assert.InRange(Math.Abs(hold[LockingArm.Shoulder]), 0.0, 1e-12);
        Assert.InRange(Math.Abs(hold[LockingArm.Elbow]), 0.0, 1e-12);
        Assert.InRange(hold.MaxAngularAcceleration, 0.0, 1e-10);
    }

    [Fact]
    public void TendonTensionsMatchAnIndependentStaticsSolution()
    {
        // From a reduced-coordinate potential-energy model of the same arm,
        // solved separately: 22.491 N at the shoulder, 7.005 N at the elbow.
        var tensions = Tensions(LockingArm.Create());

        Assert.Equal(22.491, tensions[0], 3);
        Assert.Equal(7.005, tensions[1], 3);
    }

    [Fact]
    public void TheArmStaysStraightOutWithNoTorque()
    {
        var model = LockingArm.Create();
        var state = model.CreateInitialState();

        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), model.System), 1e-4, 20_000);

        Assert.InRange(Math.Abs(model.System.GetPose(state.Values, 0).Angle), 0.0, 1e-10);
        Assert.InRange(Math.Abs(model.System.GetPose(state.Values, 1).Angle), 0.0, 1e-10);
    }

    [Fact]
    public void RestLengthScalesLengthenOnlyTheirTendon()
    {
        var exact = LockingArm.Create().System.Cables;
        var longer = LockingArm.Create(elbowRestLengthScale: 1.1).System.Cables;

        Assert.Equal(exact[0].RestLength, longer[0].RestLength);
        Assert.Equal(1.1 * exact[1].RestLength, longer[1].RestLength, 15);
    }

    [Fact]
    public void RejectsPosesATendonCannotHold()
    {
        // Lower link pointing up and back: gravity now pulls it the way the tendon does.
        Assert.Throws<ArgumentException>(() => LockingArm.Create(lowerAngle: 2.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => LockingArm.Create(new LockingArmGeometry(ElbowInsertion: 0.5)));
        Assert.Throws<ArgumentOutOfRangeException>(() => LockingArm.Create(elbowRestLengthScale: 0));
    }
}
