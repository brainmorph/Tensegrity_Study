using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// A link hanging from a fixed ground anchor at the origin by one cable
/// attached at its center of mass. The cable force passes through the center
/// of mass, so the link does not turn and the motion is vertical: a mass on
/// a one-sided spring. That gives closed forms for test 4 (oscillator) and
/// test 8 (slack and taut events), see <see cref="CableOscillatorTheory"/>.
/// </summary>
public static class CableOscillator
{
    public const double Mass = 2.0;
    public const double LinkLength = 0.2;
    public const double RestLength = 1.0;
    public const double AxialStiffness = 2000.0;

    /// <summary>k = EA / L0, N/m.</summary>
    public const double Stiffness = AxialStiffness / RestLength;

    public static PlanarLink CreateLink() => PlanarLink.SlenderBar("weight", Mass, LinkLength);

    public static Cable CreateCable(double damping) =>
        new("tendon", Attachment.Ground(Vector2d.Zero), Attachment.OnLink("weight", Vector2d.Zero), RestLength, AxialStiffness, damping);

    public static PlanarSystem CreateSystem(double damping = 0) =>
        new([CreateLink()], PlanarSystem.StandardGravity, cables: [CreateCable(damping)]);

    public static CableOscillatorTheory CreateTheory(double damping = 0) =>
        new(Mass, Stiffness, RestLength, -PlanarSystem.StandardGravity.Y, damping);

    /// <summary>State with the link at height <paramref name="y"/> below the anchor, moving at <paramref name="vy"/>.</summary>
    public static SystemState CreateState(PlanarSystem system, double y, double vy)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, y), 0), new Twist2d(new Vector2d(0, vy), 0));
        return state;
    }
}

/// <summary>
/// Closed-form motion of a mass m on a one-sided spring k with rest length
/// L0 under gravity g, hanging from the origin, starting at rest at the
/// bottom of its swing. With u = y − y_eq:
/// <list type="bullet">
/// <item>Taut, undamped: u = −A·cos(ω·t), ω = √(k/m).</item>
/// <item>Taut, damped (c &gt; 0, under-critical):
/// u = −A·e^(−γ·t)·(cos(ω_d·t) + (γ/ω_d)·sin(ω_d·t)), γ = c/(2m),
/// ω_d = √(ω² − γ²).</item>
/// <item>Bouncing, undamped, A &gt; δ = m·g/k: the cable goes slack when the
/// mass rises past y = −L0, flies ballistically, and catches again on the
/// way down. Periodic, with period 2·t₁ + 2·v₁/g.</item>
/// </list>
/// </summary>
public sealed class CableOscillatorTheory
{
    public CableOscillatorTheory(double mass, double stiffness, double restLength, double gravity, double damping)
    {
        Mass = mass;
        Stiffness = stiffness;
        RestLength = restLength;
        Gravity = gravity;
        Damping = damping;
    }

    public double Mass { get; }

    public double Stiffness { get; }

    public double RestLength { get; }

    /// <summary>Magnitude of gravity, m/s².</summary>
    public double Gravity { get; }

    public double Damping { get; }

    /// <summary>Natural frequency ω = √(k/m), rad/s.</summary>
    public double Omega => Math.Sqrt(Stiffness / Mass);

    /// <summary>Decay rate γ = c / (2m), 1/s.</summary>
    public double DecayRate => Damping / (2 * Mass);

    /// <summary>Damped frequency ω_d = √(ω² − γ²), rad/s.</summary>
    public double DampedOmega => Math.Sqrt((Omega * Omega) - (DecayRate * DecayRate));

    /// <summary>Static stretch δ = m·g/k, m. Amplitudes above δ make the cable go slack.</summary>
    public double StaticStretch => Mass * Gravity / Stiffness;

    /// <summary>Equilibrium height y_eq = −L0 − δ, m.</summary>
    public double EquilibriumY => -RestLength - StaticStretch;

    /// <summary>Taut motion from rest at y_eq − A (damped if c &gt; 0). Valid while the cable stays taut.</summary>
    public (double Y, double Vy) Taut(double amplitude, double t)
    {
        var g = DecayRate;
        var wd = DampedOmega;
        var decay = Math.Exp(-g * t);
        var u = -amplitude * decay * (Math.Cos(wd * t) + ((g / wd) * Math.Sin(wd * t)));
        var v = amplitude * decay * (Omega * Omega / wd) * Math.Sin(wd * t);
        return (EquilibriumY + u, v);
    }

    /// <summary>Time from the bottom until the cable reaches rest length on the way up, s.</summary>
    public double TimeToSlack(double amplitude)
    {
        RequireBouncing(amplitude);
        return Math.Acos(-StaticStretch / amplitude) / Omega;
    }

    /// <summary>Upward speed when the cable goes slack, m/s.</summary>
    public double SlackSpeed(double amplitude) => amplitude * Omega * Math.Sin(Omega * TimeToSlack(amplitude));

    /// <summary>Period of the bouncing motion, s.</summary>
    public double BouncePeriod(double amplitude) =>
        (2 * TimeToSlack(amplitude)) + (2 * SlackSpeed(amplitude) / Gravity);

    /// <summary>Undamped bouncing motion from rest at y_eq − A, with A &gt; δ.</summary>
    public (double Y, double Vy) Bouncing(double amplitude, double t)
    {
        RequireBouncing(amplitude);
        if (Damping != 0)
        {
            throw new InvalidOperationException("The bouncing closed form is undamped only.");
        }

        var t1 = TimeToSlack(amplitude);
        var v1 = SlackSpeed(amplitude);
        var period = BouncePeriod(amplitude);
        var tau = t - (Math.Floor(t / period) * period);
        var w = Omega;

        if (tau <= t1)
        {
            return (EquilibriumY - (amplitude * Math.Cos(w * tau)), amplitude * w * Math.Sin(w * tau));
        }

        var flight = tau - t1;
        if (flight <= 2 * v1 / Gravity)
        {
            return (-RestLength + (v1 * flight) - (0.5 * Gravity * flight * flight), v1 - (Gravity * flight));
        }

        var back = period - tau;
        return (EquilibriumY - (amplitude * Math.Cos(w * back)), -amplitude * w * Math.Sin(w * back));
    }

    private void RequireBouncing(double amplitude)
    {
        if (!(amplitude > StaticStretch))
        {
            throw new ArgumentOutOfRangeException(nameof(amplitude), amplitude, $"Must exceed the static stretch {StaticStretch} to go slack.");
        }
    }
}

/// <summary>
/// Tests 4, 7 and 8 (PLAN.md section 7) reference run: the hanging cable
/// with damping, released from 3 static stretches below equilibrium, so
/// every period passes through all three cable modes: taut, recoiling
/// (stretched but unloading faster than the spring) and slack. Protects the
/// cable law, event splitting and energy accounting together.
/// </summary>
public sealed class CableOscillatorScenario : IReferenceScenario
{
    public const double Dt = 1e-4;
    public const long Steps = 20_000;
    public const long Stride = 1_000;
    public const double Damping = 8.0;
    public const double AmplitudeOverStretch = 3.0;

    public string Name => "cable_oscillator";

    public string Description =>
        "2 kg link hanging from a ground anchor by one cable at its center of mass (L0 1 m, EA 2000 N, c 8 N s/m), released at rest 3 static stretches below equilibrium, standard gravity. Bounces through taut, recoiling and slack modes. RK4 with event splitting (1e-12 s), dt 1e-4 s, 2 s, sampled every 0.1 s.";

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var system = CableOscillator.CreateSystem(Damping);
        var theory = CableOscillator.CreateTheory(Damping);
        var state = CableOscillator.CreateState(system, theory.EquilibriumY - (AmplitudeOverStretch * theory.StaticStretch), 0.0);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var recorder = new SampleRecorder();
        var tension = new double[1];
        var initialEnergy = system.TotalEnergy(state.Values);
        var maxBalanceError = 0.0;

        FixedStepRunner.Run(state, stepper, Dt, Steps, (step, s) =>
        {
            maxBalanceError = Math.Max(maxBalanceError, Math.Abs(system.TotalEnergy(s.Values) - initialEnergy));
            if (step % Stride != 0)
            {
                return;
            }

            system.GetCableTensions(s.Values, tension);
            recorder.Add("time", s.Time);
            recorder.Add("y", system.GetPose(s.Values, 0).Position.Y);
            recorder.Add("vy", system.GetTwist(s.Values, 0).Linear.Y);
            recorder.Add("tension", tension[0]);
            recorder.Add("kinetic_energy", system.KineticEnergy(s.Values));
            recorder.Add("potential_energy", system.GravitationalPotentialEnergy(s.Values));
            recorder.Add("elastic_energy", system.CableElasticEnergy(s.Values));
            recorder.Add("dissipated_energy", system.DissipatedEnergy(s.Values));
        });

        recorder.Add("max_energy_balance_error", maxBalanceError);
        recorder.Add("event_count", stepper.EventCount);
        recorder.Add("unresolved_steps", stepper.UnresolvedSteps);
        return recorder.ToOutput();
    }
}
