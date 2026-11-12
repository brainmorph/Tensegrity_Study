using Tensegrity.Core.Integration;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// The four-bar toggle clamp (PLAN.md M8): an over-center lock that its
/// load holds shut. The load is an elastic band the clamp holds stretched.
/// <see cref="FourBarStatics"/> predicts the toggle and the crank torque
/// that balances the band, from Freudenstein's equation and virtual work,
/// independently of the simulator and the generator. Past the toggle the
/// band drives the crank into its stop, so the clamp holds with no torque
/// at any tension, and a tiny twitch on the crank releases it.
/// </summary>
public class FourBarClampTests
{
    private const double Dt = 1e-4;
    private const double ReleaseStart = 0.10037;

    private static readonly FourBarClampGeometry Geometry = new();
    private static readonly FourBarStatics Statics = new(Geometry);
    private static readonly double Toggle = FourBarClamp.ToggleCrankAngle(Geometry);
    private static readonly (double Crank, double Rocker) Latch = Statics.Latched();

    private static double Radians(double degrees) => degrees * Math.PI / 180;

    private static double Degrees(double radians) => radians * 180 / Math.PI;

    private static double RockerAt(double crank) => Statics.RockerAngle(crank, FourBarClamp.ToggleRockerAngle(Geometry));

    /// <summary>Runs the clamp for 1 s and returns its crank angle at the end and the twitch's work.</summary>
    private static (double Crank, double Work) Run(PlanarModel model)
    {
        var state = model.CreateInitialState();
        FixedStepRunner.Run(state, new EventSplittingStepper(new Rk4Integrator(), model.System), Dt, 10_000, (_, _) => { });
        return (model.System.GetJointAngle(state.Values, 0).Angle, model.System.ActuatorWork(state.Values));
    }

    private static PlanarModel Released(double tension, double impulse) =>
        FourBarClamp.Create(tension, actuators: [FourBarClamp.Twitch(-impulse, ReleaseStart, 0.02)]);

    [Fact]
    public void TheGeneratorAndFreudensteinAgreeOnTheLinkageAndTheToggle()
    {
        var (crank, rocker) = Statics.Toggle(0.5, 1.6);

        Assert.Equal(Toggle, crank, 1e-12);
        Assert.Equal(FourBarClamp.ToggleRockerAngle(Geometry), rocker, 1e-12);
        Assert.InRange(Math.Abs(Statics.Ratio(crank, rocker)), 0.0, 1e-12);
        foreach (var offset in new[] { -60.0, -10.0, -1.0, 1.0, 3.0, 20.0 })
        {
            var theta = Toggle + Radians(offset);
            Assert.Equal(FourBarClamp.RockerAngle(theta, Geometry), RockerAt(theta), 1e-12);
        }

        // Measured: the crank lines up with the coupler at 27.01°, the rocker standing at 92.29°.
        Assert.InRange(Degrees(Toggle), 27.01, 27.02);
    }

    [Theory]
    [InlineData(-10.0)]
    [InlineData(-3.0)]
    [InlineData(-1.0)]
    [InlineData(0.0)]
    [InlineData(1.0)]
    [InlineData(2.0)]
    public void TheHoldingTorqueIsTheVirtualWorkOfTheBandAndVanishesAtTheToggle(double offset)
    {
        // Clear of the closing stop, which starts 2.79° past the toggle at 50 N.
        const double tension = 50;
        var theta = Toggle + Radians(offset);
        var model = FourBarClamp.Create(tension, crankAngle: theta);

        var simulated = model.System.SolveHoldingTorques(0, model.InitialState, [FourBarClamp.CrankPivot])[FourBarClamp.CrankPivot];
        var predicted = Statics.BandHoldingTorque(theta, RockerAt(theta), tension);

        Assert.Equal(predicted, simulated, 1e-12);

        // Short of the toggle the crank must be held against the band; past
        // it the band pulls it on, toward its stop.
        Assert.True(offset switch { < 0 => simulated > 0, > 0 => simulated < 0, _ => Math.Abs(simulated) < 1e-12 });
    }

    [Theory]
    [InlineData(10.0)]
    [InlineData(50.0)]
    [InlineData(200.0)]
    public void PastTheToggleTheClampHoldsWithNoTorque(double tension)
    {
        var model = FourBarClamp.Create(tension);
        var system = model.System;
        var stops = new double[4];
        system.GetLimitTorques(model.InitialState, stops);

        // Exact equilibrium: no torque needed at any pin, and the stop
        // carries what the statics say the band puts on the crank.
        var hold = system.SolveHoldingTorques(0, model.InitialState, [FourBarClamp.CrankPivot, FourBarClamp.Knuckle, FourBarClamp.ClampPin, FourBarClamp.RockerPivot]);
        foreach (var pin in new[] { FourBarClamp.CrankPivot, FourBarClamp.Knuckle, FourBarClamp.ClampPin, FourBarClamp.RockerPivot })
        {
            Assert.InRange(Math.Abs(hold[pin]), 0.0, 1e-12);
        }

        Assert.Equal(Statics.BandHoldingTorque(Latch.Crank, Latch.Rocker, tension), stops[0], 1e-12);
        Assert.True(stops[0] < 0);

        var (crank, _) = Run(model);
        Assert.Equal(Latch.Crank, crank, 1e-12);
    }

    [Fact]
    public void ATighterBandLocksItHarder()
    {
        // The pose is the same at every tension, so the stop's torque is
        // exactly proportional to it. Measured: 0.184 N m at 50 N.
        var stops10 = new double[4];
        var stops200 = new double[4];
        var light = FourBarClamp.Create(10);
        var heavy = FourBarClamp.Create(200);
        light.System.GetLimitTorques(light.InitialState, stops10);
        heavy.System.GetLimitTorques(heavy.InitialState, stops200);

        Assert.Equal(20, stops200[0] / stops10[0], 1e-9);
    }

    /// <summary>True if the crank rests on its open stop: within 4° past it (the band presses it in) and 1° short of it.</summary>
    private static bool IsOpen(double crank) =>
        crank > Toggle - Geometry.OpenStop - Radians(4) && crank < Toggle - Geometry.OpenStop + Radians(1);

    [Fact]
    public void ShortOfTheToggleTheBandThrowsTheClampOpen()
    {
        var model = FourBarClamp.Create(50, crankAngle: Toggle - Radians(3));

        var (crank, _) = Run(model);

        Assert.True(IsOpen(crank), $"The crank ended {Degrees(crank - Toggle):F1}° from the toggle.");
    }

    [Fact]
    public void TheBandHoldsTheReleasedClampOpen()
    {
        // At the open stop, 30° short of the toggle, the band is still
        // stretched, so it presses the crank into that stop: the clamp's
        // second rest pose. Measured: 25.7 N of the 50 N latched.
        var open = Toggle - Geometry.OpenStop;
        var (fx, fy) = Statics.BandForce(RockerAt(open), 50);
        var model = FourBarClamp.Create(50, crankAngle: open);

        Assert.InRange(Math.Sqrt((fx * fx) + (fy * fy)), 25.6, 25.8);
        Assert.True(Statics.BandHoldingTorque(open, RockerAt(open), 50) > 0);
        Assert.True(model.System.SolveHoldingTorques(0, model.InitialState, [FourBarClamp.CrankPivot])[FourBarClamp.CrankPivot] > 0);
        Assert.True(IsOpen(Run(model).Crank));
    }

    [Theory]
    [InlineData(10.0, 7.8259e-4)]
    [InlineData(50.0, 2.4971e-3)]
    [InlineData(200.0, 6.4573e-3)]
    public void ATinyTwitchReleasesIt(double tension, double threshold)
    {
        // Measured release thresholds for a 20 ms twitch pulling the crank
        // back. The latched clamp holds 200 N with no torque; releasing it
        // takes 0.0065 N m s.
        var (under, _) = Run(Released(tension, 0.9 * threshold));
        var (over, work) = Run(Released(tension, 1.1 * threshold));

        Assert.Equal(Latch.Crank, under, 1e-4);
        Assert.True(IsOpen(over), $"The crank ended {Degrees(over - Toggle):F1}° from the toggle.");

        // Releasing takes at least the energy to stretch the band back to
        // the toggle, less what the compressed stop gives back.
        var stops = new double[4];
        var latched = FourBarClamp.Create(tension);
        latched.System.GetLimitTorques(latched.InitialState, stops);
        var barrier = Statics.BandEnergy(RockerAt(Toggle), tension) - Statics.BandEnergy(Latch.Rocker, tension) - (stops[0] * stops[0] / (2 * Geometry.StopStiffness));
        Assert.True(work > barrier, $"The twitch did {work} J against a barrier of {barrier} J.");

        // Measured: 1.4-3.3 times the barrier, but only 1.3-2.3% of the
        // energy in the band when latched (0.1 J at 10 N, 2 J at 200 N).
        Assert.InRange(work / Statics.BandEnergy(Latch.Rocker, tension), 0.0, 0.03);
    }
}
