using Tensegrity.Core.Analysis;
using Tensegrity.Core.Integration;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>
/// The tendon-held five-bar's statics (brainstorm-multistable-servo.md,
/// Spec), checked in closed form on a hanging chain and against the
/// simulator: the same energy and tensions in any pose, zero holding
/// torque at every minimum (check 6), the same servo torque along a held
/// path, and a released minimum that stays put.
/// </summary>
public sealed class FiveBarStaticsTests
{
    private static readonly FiveBarGeometry Geometry = new();

    private static readonly Lazy<FiveBarReport> DefaultReport =
        new(() => new FiveBarStatics(Geometry).Analyze(new FiveBarSettings(GridStep: 10 * Math.PI / 180)));

    /// <summary>Assembled poses spread over the servo and passive angles, both sides of E.</summary>
    public static TheoryData<double, double, int> Poses()
    {
        var data = new TheoryData<double, double, int>();
        foreach (var (servo, passive) in new[] { (60.0, 120.0), (20.0, 95.0), (110.0, 150.0), (75.0, 80.0), (-30.0, 210.0) })
        {
            data.Add(servo, passive, 1);
            data.Add(servo, passive, -1);
        }

        return data;
    }

    [Theory]
    [MemberData(nameof(Poses))]
    public void EnergyAndTensionsMatchTheSimulator(double servoDegrees, double passiveDegrees, int side)
    {
        var statics = new FiveBarStatics(Geometry);
        Assert.True(statics.TryAssemble(Radians(servoDegrees), Radians(passiveDegrees), side, out var pose));
        var model = FiveBar.Create(Geometry, pose);
        var state = model.InitialState;

        Assert.True(model.System.PinPositionResidual(state) < 1e-12);
        var simulated = model.System.GravitationalPotentialEnergy(state) + model.System.CableElasticEnergy(state);
        Assert.Equal(simulated, statics.Energy(pose), 1e-12);
        var tensions = new double[4];
        model.System.GetCableTensions(state, tensions);
        Assert.Equal(tensions, statics.Tensions(pose), (x, y) => Math.Abs(x - y) < 1e-9);
        Assert.Equal(side, statics.Side(pose));
    }

    [Fact]
    public void TautCouplerTendonMatchesTheSimulator()
    {
        // Tendon C–E short and off-centre, so it pulls in every pose here.
        var geometry = Geometry with { TendonCE = Geometry.TendonCE with { RestLength = 0.12 }, TendonCEPosition = 0.3 };
        var statics = new FiveBarStatics(geometry);
        Assert.True(statics.TryAssemble(Radians(60), Radians(120), 1, out var start));
        foreach (var offset in new[] { 0.0, 10.0, 25.0 })
        {
            var held = statics.RelaxHeld(start with { Servo = start.Servo + Radians(offset) });
            Assert.Equal(FiveBarFlowEnd.Minimum, held.End);
            Assert.True(statics.Tensions(held.Pose)[3] > 1, "tendon C-E should be taut");
            var model = FiveBar.Create(geometry, held.Pose);
            var state = model.InitialState;

            var tensions = new double[4];
            model.System.GetCableTensions(state, tensions);
            Assert.Equal(tensions, statics.Tensions(held.Pose), (x, y) => Math.Abs(x - y) < 1e-9);
            Assert.Equal(model.System.GravitationalPotentialEnergy(state) + model.System.CableElasticEnergy(state), statics.Energy(held.Pose), 1e-12);
            var simulated = model.System.SolveHoldingTorques(0, state, [FiveBar.ServoPin]);
            Assert.True(simulated.MaxLinearAcceleration < 1e-6, $"unheld {simulated.MaxLinearAcceleration:E2} m/s2");
            Assert.Equal(simulated[FiveBar.ServoPin], statics.ServoTorque(held.Pose), 1e-9);
        }
    }

    [Fact]
    public void AssemblyFailsWhereTheCouplersCannotReach() =>
        Assert.False(new FiveBarStatics(Geometry).TryAssemble(Radians(180), Radians(0), 1, out _));

    [Fact]
    public void GravityAloneHangsTheChainAsTheClosedFormPredicts()
    {
        // Tendons far too long to pull: only gravity acts.
        var slack = Geometry with
        {
            TendonC = Geometry.TendonC with { RestLength = 10 },
            TendonD = Geometry.TendonD with { RestLength = 10 },
            TendonE = Geometry.TendonE with { RestLength = 10 },
            TendonCE = Geometry.TendonCE with { RestLength = 10 },
        };
        var report = new FiveBarStatics(slack).Analyze(new FiveBarSettings(GridStep: 15 * Math.PI / 180));

        // Equal links and masses hang symmetrically. With H the horizontal
        // tension and w a link's weight, moments about C and A give
        // tan φ1 = w / 2H for the couplers and tan φ0 = 3w / 2H for the
        // outer links (each drooping φ below horizontal), and the chain
        // spans the base: 0.2·(cos φ0 + cos φ1) = 0.2.
        var t = Bisect(x => Math.Cos(Math.Atan(3 * x)) + Math.Cos(Math.Atan(x)) - 1, 0, 10);
        var outer = Math.Atan(3 * t);
        var inner = Math.Atan(t);

        var minimum = Assert.Single(report.Minima);
        Assert.Equal(-outer, minimum.Pose.Servo, 1e-9);
        Assert.Equal(-inner, minimum.Pose.CouplerC, 1e-9);
        Assert.Equal(Math.PI + outer, minimum.Pose.Passive + (minimum.Pose.Passive < 0 ? 2 * Math.PI : 0), 1e-9);
        Assert.Equal(Math.PI + inner, minimum.Pose.CouplerD + (minimum.Pose.CouplerD < 0 ? 2 * Math.PI : 0), 1e-9);

        // It hangs below the base, out of the servo's range, so every start in range leaves it.
        Assert.False(minimum.InRange);
        Assert.Equal(report.Starts, report.LeftRange);
        Assert.False(report.RangeOk);
        Assert.False(report.CountOk);
        Assert.True(minimum.SlowestFrequency > 0);
    }

    [Fact]
    public void EveryMinimumHoldsWithZeroServoTorqueInTheSimulator()
    {
        var report = DefaultReport.Value;
        Assert.True(report.Minima.Count >= 2, "The default geometry should have several minima to check.");
        foreach (var minimum in report.Minima)
        {
            var model = FiveBar.Create(Geometry, minimum.Pose);
            var state = model.InitialState;

            // Spec check 6: the servo pin holds nothing.
            var held = model.System.SolveHoldingTorques(0, state, [FiveBar.ServoPin]);
            Assert.True(Math.Abs(held[FiveBar.ServoPin]) < 1e-9, $"servo torque {held[FiveBar.ServoPin]:E2} N m at {minimum.Pose}");

            // And with no torque anywhere, nothing accelerates: the shape holds itself.
            var free = model.System.SolveHoldingTorques(0, state, []);
            Assert.True(free.MaxLinearAcceleration < 1e-7 && free.MaxAngularAcceleration < 1e-6, $"unheld {free.MaxLinearAcceleration:E2} m/s2, {free.MaxAngularAcceleration:E2} rad/s2");
            Assert.Equal(0, new FiveBarStatics(Geometry).ServoTorque(minimum.Pose), 1e-9);
        }
    }

    [Fact]
    public void HeldServoTorqueMatchesTheSimulator()
    {
        var statics = new FiveBarStatics(Geometry);
        var start = DefaultReport.Value.Minima[0].Pose;
        foreach (var offset in new[] { -8.0, 4.0, 15.0, 25.0 })
        {
            var held = statics.RelaxHeld(start with { Servo = start.Servo + Radians(offset) });
            Assert.Equal(FiveBarFlowEnd.Minimum, held.End);
            var model = FiveBar.Create(Geometry, held.Pose);

            var simulated = model.System.SolveHoldingTorques(0, model.InitialState, [FiveBar.ServoPin]);
            Assert.True(simulated.MaxLinearAcceleration < 1e-7, $"unheld {simulated.MaxLinearAcceleration:E2} m/s2 at offset {offset}");
            Assert.Equal(simulated[FiveBar.ServoPin], statics.ServoTorque(held.Pose), 1e-9);
        }
    }

    [Fact]
    public void ReleasedAtAMinimumTheSimulatorStaysPut()
    {
        var minimum = DefaultReport.Value.Minima[^1];
        var model = FiveBar.Create(Geometry, minimum.Pose);
        var state = model.CreateInitialState();
        var stepper = new EventSplittingStepper(new Rk4Integrator(), model.System);
        var before = Enumerable.Range(0, 4).Select(b => model.System.GetPose(state.Values, b).Angle).ToArray();

        FixedStepRunner.Run(state, stepper, 1e-4, 2_000);

        for (var b = 0; b < 4; b++)
        {
            Assert.True(Math.Abs(model.System.GetPose(state.Values, b).Angle - before[b]) < 1e-6, $"link {b} drifted");
        }
    }

    [Fact]
    public void TransitionsStartAtRestAndEndAtTheTargetAngle()
    {
        var report = DefaultReport.Value;
        Assert.NotEmpty(report.Transitions);
        foreach (var transition in report.Transitions)
        {
            var from = report.Minima[transition.From].Pose;
            var to = report.Minima[transition.To].Pose;
            Assert.Equal(from.Servo, transition.Path[0].Servo);
            Assert.Equal(0, transition.Path[0].Torque, 1e-9);
            if (transition.BlockedAt is null)
            {
                Assert.Equal(to.Servo, transition.Path[^1].Servo, 1e-12);
            }

            if (transition.Arrived)
            {
                Assert.Equal(to.Passive, transition.Path[^1].Passive, 1e-6);
                Assert.Equal(0, transition.Path[^1].Torque, 1e-8);
            }

            // Steps no larger than the path step.
            for (var i = 1; i < transition.Path.Count; i++)
            {
                Assert.True(Math.Abs(transition.Path[i].Servo - transition.Path[i - 1].Servo) <= report.Settings.PathStep + 1e-12);
            }
        }
    }

    [Fact]
    public void TheAnalysisIsDeterministic()
    {
        var settings = new FiveBarSettings(GridStep: 15 * Math.PI / 180);
        var first = new FiveBarStatics(Geometry).Analyze(settings);
        var second = new FiveBarStatics(Geometry).Analyze(settings);

        Assert.Equal(first.Minima.Select(m => m.Pose), second.Minima.Select(m => m.Pose));
        Assert.Equal(first.Minima.Select(m => m.BasinStarts), second.Minima.Select(m => m.BasinStarts));
        Assert.Equal(first.Transitions.Select(t => t.PeakTorque), second.Transitions.Select(t => t.PeakTorque));
    }

    [Fact]
    public void ParametersSetAbsoluteValues()
    {
        var g = FiveBarParameter.ApplyAll(Geometry, "tendon_e.y=0.6, tendon_c.rest=0.2,base_width=0.5");

        Assert.Equal(0.6, g.TendonE.Anchor.Y);
        Assert.Equal(Geometry.TendonE.Anchor.X, g.TendonE.Anchor.X);
        Assert.Equal(0.2, g.TendonC.RestLength);
        Assert.Equal(0.5, g.BaseWidth);
        Assert.All(FiveBarParameter.Names, name => FiveBarParameter.Parse(name).Apply(Geometry, 0.123).Validate());
        Assert.Throws<FormatException>(() => FiveBarParameter.ApplyAll(Geometry, "tendon_f.x=1"));
        Assert.Throws<FormatException>(() => FiveBarParameter.ApplyAll(Geometry, "tendon_c.x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => FiveBarParameter.ApplyAll(Geometry, "tendon_c.rest=-1").Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => FiveBarParameter.ApplyAll(Geometry, "tendon_ce.at=1.5").Validate());
        Assert.Equal(0.25, FiveBarParameter.ApplyAll(Geometry, "tendon_ce.at=0.25").TendonCEPosition);
    }

    private static double Radians(double degrees) => degrees * Math.PI / 180;

    private static double Bisect(Func<double, double> f, double low, double high)
    {
        var fLow = f(low);
        for (var i = 0; i < 200; i++)
        {
            var mid = 0.5 * (low + high);
            var fMid = f(mid);
            if (Math.Sign(fMid) == Math.Sign(fLow))
            {
                (low, fLow) = (mid, fMid);
            }
            else
            {
                high = mid;
            }
        }

        return 0.5 * (low + high);
    }
}
