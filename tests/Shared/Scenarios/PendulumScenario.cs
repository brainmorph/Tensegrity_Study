using Tensegrity.Core.Bodies;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// A link pinned to the ground at its <see cref="PlanarLink.EndA"/>, in
/// reduced coordinates: state is [θ, θ̇], where θ is the link angle.
///
/// TEST-SIDE STAND-IN until M2. Pin constraints (Lagrange multipliers) are
/// M2 work, so in M1 the pendulum is integrated in its single angle using
/// the Core RK4 and the Core link inertia. <see cref="WriteFullState"/>
/// reconstructs the maximal-coordinate state so the M2 pinned version can be
/// checked against the same baseline.
///
/// Equation of motion: I_pivot·θ̈ = r × (m·g), r = R(θ)·(−EndA) the center of
/// mass offset from the pivot.
/// </summary>
public sealed class ReducedPendulum
{
    public ReducedPendulum(PlanarLink link, Vector2d pivot, Vector2d gravity)
    {
        Link = link;
        Pivot = pivot;
        Gravity = gravity;
        PivotInertia = link.MomentOfInertiaAbout(link.EndA);
        PivotToCenter = -link.EndA;
    }

    public PlanarLink Link { get; }

    public Vector2d Pivot { get; }

    public Vector2d Gravity { get; }

    /// <summary>Moment of inertia about the pivot, kg·m².</summary>
    public double PivotInertia { get; }

    /// <summary>Body-frame offset from pivot to center of mass.</summary>
    public Vector2d PivotToCenter { get; }

    /// <summary>Pivot-to-center distance d, m.</summary>
    public double Distance => PivotToCenter.Length;

    /// <summary>Link angle at which the pendulum hangs at rest.</summary>
    public double HangingAngle => Math.Atan2(Gravity.Y, Gravity.X) - Math.Atan2(PivotToCenter.Y, PivotToCenter.X);

    /// <summary>Small-amplitude period, T = 2π·√(I_pivot / (m·g·d)).</summary>
    public double SmallAmplitudePeriod => 2 * Math.PI * Math.Sqrt(PivotInertia / (Link.Mass * Gravity.Length * Distance));

    /// <summary>
    /// Exact period for a swing of the given amplitude from hanging:
    /// T = 4·√(I_pivot / (m·g·d))·K(sin(amplitude/2)), with K the complete
    /// elliptic integral of the first kind.
    /// </summary>
    public double ExactPeriod(double amplitude) =>
        4 * Math.Sqrt(PivotInertia / (Link.Mass * Gravity.Length * Distance))
        * CompleteEllipticK(Math.Sin(0.5 * amplitude));

    public void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative)
    {
        var r = PivotToCenter.Rotated(state[0]);
        derivative[0] = state[1];
        derivative[1] = Vector2d.Cross(r, Link.Mass * Gravity) / PivotInertia;
    }

    /// <summary>Writes the equivalent maximal-coordinate state into a one-body <paramref name="system"/>.</summary>
    public void WriteFullState(PlanarSystem system, ReadOnlySpan<double> reduced, Span<double> full)
    {
        var theta = reduced[0];
        var omega = reduced[1];
        var r = PivotToCenter.Rotated(theta);
        system.SetBody(full, 0, new Pose2d(Pivot + r, theta), new Twist2d(Vector2d.Cross(omega, r), omega));
    }

    /// <summary>K(k) by the arithmetic–geometric mean: K = π / (2·AGM(1, √(1 − k²))).</summary>
    public static double CompleteEllipticK(double k)
    {
        if (!(Math.Abs(k) < 1))
        {
            throw new ArgumentOutOfRangeException(nameof(k), k, "Modulus must satisfy |k| < 1.");
        }

        var a = 1.0;
        var b = Math.Sqrt(1 - (k * k));
        for (var i = 0; i < 64 && Math.Abs(a - b) > 1e-16 * a; i++)
        {
            (a, b) = (0.5 * (a + b), Math.Sqrt(a * b));
        }

        return Math.PI / (2 * a);
    }
}

/// <summary>
/// Test 3 (PLAN.md section 7) reference run: a 1 m, 1 kg slender bar pinned at
/// one end, released from rest 1 rad from hanging. Large amplitude on purpose,
/// so the nonlinear term matters and any change in the dynamics shows.
/// </summary>
public sealed class PendulumScenario : IReferenceScenario
{
    public const double Dt = 1e-4;
    public const long Steps = 20_000;
    public const long Stride = 2_000;
    public const double Amplitude = 1.0;

    public string Name => "pendulum";

    public string Description =>
        "Slender bar (1 kg, 1 m) pinned at one end to the origin, released from rest 1 rad from hanging, standard gravity. Reduced-coordinate stand-in until M2 pins. RK4, dt 1e-4 s, 2 s, sampled every 0.2 s.";

    public static PlanarLink CreateLink() => PlanarLink.SlenderBar("bar", mass: 1.0, length: 1.0);

    public static ReducedPendulum CreatePendulum() => new(CreateLink(), Vector2d.Zero, PlanarSystem.StandardGravity);

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var pendulum = CreatePendulum();
        var system = new PlanarSystem([pendulum.Link], pendulum.Gravity);
        var full = new double[system.StateLength];
        var state = new SystemState([pendulum.HangingAngle + Amplitude, 0.0]);
        var recorder = new SampleRecorder();

        pendulum.WriteFullState(system, state.Values, full);
        var initialEnergy = system.KineticEnergy(full) + system.GravitationalPotentialEnergy(full);
        var maxDrift = 0.0;

        FixedStepRunner.Run(state, new Rk4Integrator(), pendulum.Evaluate, Dt, Steps, (step, s) =>
        {
            pendulum.WriteFullState(system, s.Values, full);
            var kinetic = system.KineticEnergy(full);
            var potential = system.GravitationalPotentialEnergy(full);
            maxDrift = Math.Max(maxDrift, Math.Abs(kinetic + potential - initialEnergy));

            if (step % Stride != 0)
            {
                return;
            }

            var pose = system.GetPose(full, 0);
            recorder.Add("time", s.Time);
            recorder.Add("x", pose.Position.X);
            recorder.Add("y", pose.Position.Y);
            recorder.Add("theta", pose.Angle);
            recorder.Add("omega", system.GetTwist(full, 0).Angular);
            recorder.Add("kinetic_energy", kinetic);
            recorder.Add("potential_energy", potential);
        });

        recorder.Add("max_energy_drift", maxDrift);
        return recorder.ToOutput();
    }
}
