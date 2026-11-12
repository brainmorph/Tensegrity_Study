using System.Text.Json;
using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Output;
using Tensegrity.Core.Systems;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// The series-elastic muscle (PLAN.md M9, stage 1): a contractile part that
/// shortens at a constant speed while active, in series with an elastic
/// tendon. A weight hangs from a ground anchor by the muscle, attached at
/// its center of mass so it moves only vertically.
/// </summary>
public class MuscleTests
{
    private const double Mass = 2.0;
    private const double TendonRest = 0.5;
    private const double TendonEA = 2000.0;
    private const double TendonDamping = 100.0;
    private const double InitialContractile = 0.5;
    private const double Speed = 0.2;
    private const double Dt = 1e-4;
    /// <summary>2.5 s: the tendon damper rings down at c/2m = 25 /s, so 1.6 s after the muscle relaxes the motion is gone.</summary>
    private const int Steps = 25000;

    /// <summary>Off until 0.1 s, ramped to full by 0.2 s, held to 0.8 s and relaxed by 0.9 s: 0.7 s at full speed in all.</summary>
    private static readonly Schedule Activation = new(
    [
        new SchedulePoint(0.1, 0),
        new SchedulePoint(0.2, 1),
        new SchedulePoint(0.8, 1),
        new SchedulePoint(0.9, 0),
    ]);

    private const double Shortening = Speed * 0.7;

    private static double G => -PlanarSystem.StandardGravity.Y;

    private static readonly PlanarLink Weight = PlanarLink.SlenderBar("weight", Mass, 0.2);

    private static Cable MuscleCable(double tendonRest = TendonRest) => new(
        "muscle",
        Attachment.Ground(Vector2d.Zero),
        Attachment.OnLink("weight", Vector2d.Zero),
        tendonRest,
        TendonEA,
        TendonDamping,
        new Muscle(InitialContractile, Speed, Activation));

    private static PlanarSystem Hanging() => new([Weight], PlanarSystem.StandardGravity, cables: [MuscleCable()]);

    /// <summary>Hanging at rest: the tendon stretched m·g/k below the relaxed muscle and tendon.</summary>
    private static double RestY => -(InitialContractile + TendonRest + (Mass * G * TendonRest / TendonEA));

    private static SystemState StateAt(PlanarSystem system, double y)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, y), 0), default);
        return state;
    }

    private static EventSplittingStepper Run(PlanarSystem system, SystemState state, Action<long, SystemState>? observe = null)
    {
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        FixedStepRunner.Run(state, stepper, Dt, Steps, (step, s) => observe?.Invoke(step, s));
        return stepper;
    }

    [Fact]
    public void ContractileLengthFollowsTheClosedForm()
    {
        var system = Hanging();
        var state = StateAt(system, RestY);
        var muscle = system.Cables[0].Muscle!;
        Assert.Equal(InitialContractile, system.GetContractileLength(state.Values, 0));
        Assert.Equal(Shortening, muscle.TotalShortening, 14);

        var worst = 0.0;
        Run(system, state, (_, s) =>
            worst = Math.Max(worst, Math.Abs(system.GetContractileLength(s.Values, 0) - muscle.LengthAt(s.Time))));

        // RK4 integrates the piecewise-quadratic ℓ(t) exactly inside each
        // activation segment, so only rounding over 25 000 steps is left.
        Assert.True(worst < 1e-12, $"Contractile length strays {worst:E2} m from the closed form.");
        Assert.Equal(InitialContractile - Shortening, system.GetContractileLength(state.Values, 0), 12);
    }

    [Fact]
    public void ShorteningLiftsTheWeightByTheTotalShortening()
    {
        var system = Hanging();
        var state = StateAt(system, RestY);
        Run(system, state);

        var tension = new double[1];
        system.GetCableTensions(state.Time, state.Values, tension);
        Assert.Equal(RestY + Shortening, system.GetPose(state.Values, 0).Position.Y, 8);
        Assert.Equal(0.0, system.GetTwist(state.Values, 0).Linear.Y, 8);

        // Settled again, the tendon carries the weight alone: the tension rose and fell back.
        Assert.Equal(Mass * G, tension[0], 6);
    }

    [Fact]
    public void EnergyBalanceHoldsAndTheMuscleDoesTheLiftingWork()
    {
        var system = Hanging();
        var state = StateAt(system, RestY);
        var initial = system.TotalEnergy(state.Values);
        var initialElastic = system.CableElasticEnergy(state.Values);
        var drift = 0.0;
        Run(system, state, (_, s) => drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial)));

        Assert.True(drift < 1e-9, $"Energy balance drifts {drift:E2} J.");

        // Back at rest with the same tendon stretch, the muscle's work went into lifting and the damper.
        var lift = Mass * G * Shortening;
        Assert.Equal(initialElastic, system.CableElasticEnergy(state.Values), 9);
        Assert.Equal(lift, system.ActuatorWork(state.Values) - system.DissipatedEnergy(state.Values), 6);
        Assert.True(system.ActuatorWork(state.Values) > lift);
    }

    /// <summary>
    /// The weight is also tied down to a ground point below it by a plain
    /// cable, so shortening the muscle stretches both springs: the muscle's
    /// tension rises by Δ·k₁·k₂/(k₁ + k₂) and the weight rises by
    /// Δ·k₁/(k₁ + k₂), springs in series.
    /// </summary>
    [Fact]
    public void ShorteningAgainstASecondSpringRaisesTheTensionQuasiStatically()
    {
        const double upperTension = 40.0;
        const double lowerEA = 1000.0;
        const double y0 = -1.0;
        const double floor = -2.0;
        var tendonRest = Cable.RestLengthFor(-y0 - InitialContractile, upperTension, TendonEA);
        var lowerTension = upperTension - (Mass * G);
        var lowerRest = Cable.RestLengthFor(y0 - floor, lowerTension, lowerEA);
        var lower = new Cable("tie", Attachment.OnLink("weight", Vector2d.Zero), Attachment.Ground(new Vector2d(0, floor)), lowerRest, lowerEA, TendonDamping);
        var system = new PlanarSystem([Weight], PlanarSystem.StandardGravity, cables: [MuscleCable(tendonRest), lower]);
        var state = StateAt(system, y0);

        var tensions = new double[2];
        system.GetCableTensions(0, state.Values, tensions);
        Assert.Equal(upperTension, tensions[0], 10);
        Assert.Equal(lowerTension, tensions[1], 10);

        Run(system, state);

        var k1 = TendonEA / tendonRest;
        var k2 = lowerEA / lowerRest;
        system.GetCableTensions(state.Time, state.Values, tensions);
        Assert.Equal(y0 + (Shortening * k1 / (k1 + k2)), system.GetPose(state.Values, 0).Position.Y, 8);
        Assert.Equal(upperTension + (Shortening * k1 * k2 / (k1 + k2)), tensions[0], 5);
        Assert.Equal(tensions[0] - (Mass * G), tensions[1], 5);
    }

    [Fact]
    public void PlainCableQueriesRefuseAMuscleWithoutATime()
    {
        var system = Hanging();
        var state = StateAt(system, RestY);
        Assert.Throws<InvalidOperationException>(() => system.GetCableTensions(state.Values, new double[1]));
        Assert.Throws<InvalidOperationException>(() => system.GetCableMode(state.Values, 0));
        Assert.Equal(CableMode.Taut, system.GetCableMode(0, state.Values, 0));
    }

    [Fact]
    public void AMuscleAddsOneStateAfterEverythingElse()
    {
        var plain = new PlanarSystem([Weight], PlanarSystem.StandardGravity, cables: [CableOscillatorLikeCable()]);
        var withMuscle = Hanging();
        Assert.Equal(0, plain.MuscleCount);
        Assert.Equal(1, withMuscle.MuscleCount);
        Assert.Equal(plain.StateLength + 1, withMuscle.StateLength);
        Assert.Equal(withMuscle.StateLength - 1, withMuscle.ContractileLengthIndex(0));
        Assert.Throws<ArgumentException>(() => plain.ContractileLengthIndex(0));
    }

    private static Cable CableOscillatorLikeCable() =>
        new("tendon", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("weight", Vector2d.Zero), 1.0, TendonEA, TendonDamping);

    [Theory]
    [InlineData(1.5)]
    [InlineData(-0.1)]
    public void ActivationMustLieBetweenZeroAndOne(double peak) =>
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new Muscle(1.0, 0.1, new Schedule([new SchedulePoint(0, 0), new SchedulePoint(1, peak), new SchedulePoint(2, 0)])));

    [Fact]
    public void ActivationMustEndAtZero() =>
        Assert.Throws<ArgumentException>(() => new Muscle(1.0, 0.1, Schedule.Ramp(0, 0, 1, 1)));

    [Fact]
    public void ActivationTimesMustNotBeNegative() =>
        Assert.Throws<ArgumentOutOfRangeException>(() => new Muscle(1.0, 0.1, Schedule.Ramp(-1, 0, 1, 0)));

    [Fact]
    public void TheMuscleCannotShortenPastZero()
    {
        // 1 s at full speed 0.2 m/s is 0.2 m, more than 0.15 m.
        var schedule = new Schedule([new SchedulePoint(0, 1), new SchedulePoint(1, 1), new SchedulePoint(1.000001, 0)]);
        Assert.Throws<ArgumentException>(() => new Muscle(0.15, 0.2, schedule));
        Assert.Equal(0.2, new Muscle(0.25, 0.2, schedule).TotalShortening, 6);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-0.1)]
    [InlineData(double.NaN)]
    public void SpeedAndLengthMustBePositive(double bad)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new Muscle(bad, 0.1, Activation));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Muscle(1.0, bad, Activation));
    }

    [Fact]
    public void ARelaxedMuscleKeepsItsLength()
    {
        var muscle = new Muscle(0.3, 0.2, Schedule.Constant(0));
        Assert.Equal(0.0, muscle.TotalShortening);
        Assert.Equal(0.3, muscle.LengthAt(10));
    }

    [Fact]
    public void AMuscleSurvivesATopologyRoundTrip()
    {
        var system = Hanging();
        var state = StateAt(system, RestY);
        var text = RunFiles.SerializeTopology(TopologyMapping.ToDocument(system, state.Values, "muscle_test"));
        Assert.Contains("\"shortening_speed\"", text, StringComparison.Ordinal);

        var document = JsonSerializer.Deserialize<TopologyDocument>(text, RunFiles.Json)!;
        var (rebuilt, rebuiltState) = TopologyMapping.FromDocument(document);
        var muscle = rebuilt.Cables[0].Muscle!;
        Assert.Equal(InitialContractile, muscle.InitialLength);
        Assert.Equal(Speed, muscle.ShorteningSpeed);
        Assert.Equal(Activation.Points, muscle.Activation.Points);
        Assert.Equal(TendonRest, rebuilt.Cables[0].RestLength);
        Assert.Equal(state.Values, rebuiltState);
    }

    [Fact]
    public void APlainCableWritesNoMuscle()
    {
        var system = new PlanarSystem([Weight], PlanarSystem.StandardGravity, cables: [CableOscillatorLikeCable()]);
        var text = RunFiles.SerializeTopology(TopologyMapping.ToDocument(system, system.CreateState().Values, "plain"));
        Assert.DoesNotContain("muscle", text, StringComparison.Ordinal);
    }
}
