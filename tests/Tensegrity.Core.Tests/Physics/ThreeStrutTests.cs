using Tensegrity.Core.Integration;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;
using Xunit.Abstractions;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// M9: the <see cref="ThreeStrut"/> frame snaps through when its muscle
/// shortens. The statics (<see cref="ThreeStrutStatics"/>, independent of
/// the simulator) predict where the starting shape gives way and the shape
/// it snaps to; the simulator must agree.
/// </summary>
public class ThreeStrutTests(ITestOutputHelper output)
{
    private static readonly ThreeStrutGeometry Geometry = new();
    private static readonly PlanarModel Model = ThreeStrut.Create(Geometry);
    private static readonly ThreeStrutStatics Statics = new(Geometry, Model.System.Cables);
    private static readonly Lazy<ThreeStrutSnap> Snap = new(() => Statics.FindSnap());

    private const double Dt = 1e-4;

    /// <summary>10 s: 0.5 s at rest, about 4 s of shortening, the snap, and the ring-down.</summary>
    private const int Steps = 100_000;

    private static double ToDegrees(double radians) => radians * 180 / Math.PI;

    private static double Lean(PlanarSystem system, ReadOnlySpan<double> state, int post) =>
        (Math.PI / 2) - system.GetPose(state, post).Angle;

    [Fact]
    public void TheStartingShapeIsAnExactStableEquilibrium()
    {
        var start = Statics.Start;
        var gradient = Statics.Gradient(start, 0);
        Assert.True(gradient.Max(Math.Abs) < 1e-10, $"Gradient {gradient.Max(Math.Abs):E2} J/rad.");
        Assert.True(Statics.SmallestEigenvalue(start, 0) > 0);
        Assert.Equal(Geometry.MuscleTension, Statics.MuscleTension(start, 0), 10);
        Assert.True(Statics.Tension(ThreeStrut.Stay, start, 0) > 0);
        Assert.Equal(0.0, Statics.Tension(ThreeStrut.Guy, start, 0));

        // The simulator agrees: no torque holds it.
        var state = Model.CreateInitialState();
        var hold = Model.System.SolveHoldingTorques(0, state.Values, ["foot1", "foot2", "foot3"]);
        Assert.True(hold.Torques.Max(Math.Abs) < 1e-10, $"Holding torques up to {hold.Torques.Max(Math.Abs):E2} N·m.");
    }

    [Fact]
    public void ShorteningRaisesTheTensionUntilTheStayGoesSlack()
    {
        var snap = Snap.Value;
        output.WriteLine($"snap at {snap.Shortening:F9} m of shortening; muscle {snap.MuscleTension:F6} N; leans {string.Join(" ", snap.Leans.Select(a => ToDegrees(a).ToString("F4")))} deg");

        // The tension rises almost fourfold while the frame barely moves.
        Assert.True(snap.MuscleTension > 3.5 * Geometry.MuscleTension);
        Assert.All(snap.Leans, a => Assert.InRange(ToDegrees(a), -15.0, -14.5));

        // The branch ends where the stay goes slack, and the muscle alone cannot hold the frame up.
        Assert.True(Statics.Tension(ThreeStrut.Stay, snap.Leans, snap.Shortening) < 1e-6);
        var unbraced = Statics.SmallestEigenvalue(snap.Leans, snap.Shortening, without: ThreeStrut.Stay);
        output.WriteLine($"smallest stiffness without the stay: {unbraced:F4} J/rad^2");
        Assert.True(unbraced < 0);

        // The tendon stays under 8% strain, as M8 required of a realistic tendon.
        var tendon = Model.System.Cables.Single(c => c.Muscle is not null);
        Assert.True(snap.MuscleTension / tendon.AxialStiffness < 0.08);
    }

    [Fact]
    public void TheSnappedShapeHoldsWithTheMuscleAtFivePercent()
    {
        var snap = Snap.Value;
        var target = 0.05 * Geometry.MuscleTension;
        var stop = Statics.StopFor(target, snap);
        output.WriteLine($"stop for 5%: {stop:F12} m of shortening (geometry has {Geometry.Shortening})");

        var shortening = Geometry.Shortening;
        var snapped = Statics.Snapped(shortening);
        output.WriteLine($"snapped: leans {string.Join(" ", snapped.Select(a => ToDegrees(a).ToString("F4")))} deg; muscle {Statics.MuscleTension(snapped, shortening):F6} N; guy {Statics.Tension(ThreeStrut.Guy, snapped, shortening):F6} N");

        Assert.True(shortening > snap.Shortening, "The muscle must shorten past the snap.");
        Assert.All(snapped, a => Assert.InRange(ToDegrees(a), 14.5, 15.5));
        Assert.True(Statics.SmallestEigenvalue(snapped, shortening) > 0);
        Assert.Equal(0.0, Statics.Tension(ThreeStrut.Stay, snapped, shortening));
        Assert.True(Statics.Tension(ThreeStrut.Guy, snapped, shortening) > 0);
        Assert.Equal(target, Statics.MuscleTension(snapped, shortening), 4);
    }

    [Fact]
    public void TheSimulatorSnapsWhereTheStaticsPredict()
    {
        var system = Model.System;
        var state = Model.CreateInitialState();
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var muscle = system.Cables.Select((c, i) => (c, i)).Single(x => x.c.Muscle is not null).i;
        var stay = system.Cables.Select((c, i) => (c, i)).Single(x => x.c.Id == ThreeStrut.Stay).i;
        var tensions = new double[system.Cables.Count];
        var initial = system.TotalEnergy(state.Values);
        var drift = 0.0;
        var restless = 0.0;
        var slackAt = double.NaN;

        FixedStepRunner.Run(state, stepper, Dt, Steps, (_, s) =>
        {
            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial));
            if (s.Time < Geometry.ActivationStart)
            {
                for (var p = 0; p < 3; p++)
                {
                    restless = Math.Max(restless, Math.Abs(Lean(system, s.Values, p) + Geometry.Lean));
                }
            }

            system.GetCableTensions(s.Time, s.Values, tensions);
            if (double.IsNaN(slackAt) && tensions[stay] <= 0)
            {
                slackAt = Geometry.ContractileLength - system.GetContractileLength(s.Values, muscle);
            }
        });

        var snap = Snap.Value;
        var predicted = Statics.Snapped(Geometry.Shortening);
        system.GetCableTensions(state.Time, state.Values, tensions);
        var leans = Enumerable.Range(0, 3).Select(p => Lean(system, state.Values, p)).ToArray();
        output.WriteLine($"stay slack at {slackAt:F6} m of shortening (statics {snap.Shortening:F6})");
        output.WriteLine($"end: leans {string.Join(" ", leans.Select(a => ToDegrees(a).ToString("F6")))} deg; muscle {tensions[muscle]:F6} N; energy drift {drift:E2} J; events {stepper.EventCount}");

        Assert.True(restless < 1e-9, $"The starting shape moved {restless:E2} rad before the muscle woke.");
        Assert.True(drift < 1e-8, $"Energy balance drifts {drift:E2} J.");
        Assert.Equal(0, stepper.UnresolvedSteps);

        // Shortening slowly, the stay lets go where the statics say, less
        // the tendon damper's lag: stretching at about v, it adds c·v to the
        // muscle's tension, worth c·v/k of shortening.
        var tendon = system.Cables[muscle];
        var lag = tendon.Damping * Geometry.ShorteningSpeed / tendon.Stiffness;
        output.WriteLine($"damper lag {lag:F6} m; statics less lag {snap.Shortening - lag:F6} m");
        Assert.True(Math.Abs(slackAt - (snap.Shortening - lag)) < 5e-5, $"The stay let go at {slackAt:F6} m, not {snap.Shortening - lag:F6} m.");

        // The frame has snapped and come to rest in the predicted shape, with the muscle at 5%.
        Assert.Equal(Geometry.Shortening, Geometry.ContractileLength - system.GetContractileLength(state.Values, muscle), 10);
        for (var p = 0; p < 3; p++)
        {
            Assert.Equal(predicted[p], leans[p], 6);
            Assert.Equal(0.0, system.GetTwist(state.Values, p).Angular, 6);
        }

        Assert.Equal(Statics.MuscleTension(predicted, Geometry.Shortening), tensions[muscle], 5);
        var hold = system.SolveHoldingTorques(state.Time, state.Values, ["foot1", "foot2", "foot3"]);
        Assert.True(hold.Torques.Max(Math.Abs) < 1e-6, $"Holding torques up to {hold.Torques.Max(Math.Abs):E2} N·m.");
    }
}
