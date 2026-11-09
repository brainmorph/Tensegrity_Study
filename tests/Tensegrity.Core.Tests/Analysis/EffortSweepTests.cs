using Tensegrity.Core.Analysis;
using Tensegrity.RunFormat;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>Effort curves: holding torque in a fixed pose as one cable's rest length is scaled.</summary>
public class EffortSweepTests
{
    private static readonly string[] Pins = [LockingArm.Shoulder, LockingArm.Elbow];

    private static TopologyDocument KickedArm() => TopologyFiles.ToDocument(ReferenceTopologies.LockingArmKicked());

    [Fact]
    public void ElbowEffortIsItsWeightMomentLessWhatTheTendonCarries()
    {
        double[] scales = [0.98, 0.99, 1.0, 1.003, 1.006, 1.02];

        var points = EffortSweep.Run(KickedArm(), LockingArm.ElbowTendon, Pins, scales);

        // Slack at 1.02: the elbow carries the lower link's whole weight moment,
        // and with tension T the tendon takes the share T / T(1) of it.
        var full = points[^1].Torques[1];
        var design = points[2].Tension;
        Assert.Equal(0.0, points[^1].Tension);
        Assert.Equal(0.12 * 9.81 * 0.35 / 2, full, 12);
        foreach (var point in points)
        {
            Assert.Equal(full * (1 - (point.Tension / design)), point.Torques[1], 10);
            Assert.InRange(Math.Abs(point.Torques[0]), 0.0, 1e-10); // The elbow tendon is internal to the arm.
            Assert.InRange(point.MaxAngularAcceleration, 0.0, 1e-10);
        }

        Assert.InRange(Math.Abs(points[2].Torques[1]), 0.0, 1e-12);
        Assert.True(points[0].Torques[1] < 0, "A short tendon over-lifts: the joint must pull down.");
    }

    [Fact]
    public void TheSweepIgnoresInitialVelocitiesAndLeavesTheDocumentAlone()
    {
        var kicked = KickedArm();
        var before = RunFiles.SerializeTopology(kicked);
        var still = TopologyFiles.ToDocument(LockingArm.Create());

        var a = EffortSweep.Run(kicked, LockingArm.ElbowTendon, Pins, [1.004]);
        var b = EffortSweep.Run(still, LockingArm.ElbowTendon, Pins, [1.004]);

        Assert.Equal(b[0].Torques, a[0].Torques);
        Assert.Equal(b[0].Tension, a[0].Tension);
        Assert.Equal(before, RunFiles.SerializeTopology(kicked));
    }

    [Fact]
    public void UnknownCableIsAnError() =>
        Assert.Throws<ArgumentException>(() => EffortSweep.Run(KickedArm(), "wing_tendon", Pins, [1.0]));
}
