using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Closed-form results for a rigid link swinging about a fixed pivot at a
/// body-fixed point. Formulas only: the dynamics are simulated by
/// <see cref="PlanarSystem"/> with a real pin.
/// </summary>
public sealed class PendulumTheory
{
    public PendulumTheory(PlanarLink link, Vector2d pivotLocal, Vector2d pivot, Vector2d gravity)
    {
        Link = link;
        PivotLocal = pivotLocal;
        Pivot = pivot;
        Gravity = gravity;
        PivotInertia = link.MomentOfInertiaAbout(pivotLocal);
        PivotToCenter = -pivotLocal;
    }

    public PlanarLink Link { get; }

    /// <summary>Body-frame point that is pinned.</summary>
    public Vector2d PivotLocal { get; }

    /// <summary>World position of the pivot.</summary>
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

    /// <summary>
    /// Pose and twist of the link at angle <paramref name="theta"/> and rate
    /// <paramref name="omega"/>, with the pinned point exactly at the pivot.
    /// </summary>
    public (Pose2d Pose, Twist2d Twist) StateAt(double theta, double omega)
    {
        var r = PivotToCenter.Rotated(theta);
        return (new Pose2d(Pivot + r, theta), new Twist2d(Vector2d.Cross(omega, r), omega));
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
///
/// Simulated in maximal coordinates with a Lagrange-multiplier pin to the
/// ground. The baseline was recorded in M1 with a reduced one-angle model;
/// this pinned run must reproduce it unchanged.
/// </summary>
public sealed class PendulumScenario : IReferenceScenario
{
    public const double Dt = 1e-4;
    public const long Steps = 20_000;
    public const long Stride = 2_000;
    public const double Amplitude = 1.0;

    public string Name => "pendulum";

    public string Description =>
        "Slender bar (1 kg, 1 m) pinned at one end to the origin, released from rest 1 rad from hanging, standard gravity. Lagrange-multiplier pin in maximal coordinates (M1 baseline from a reduced one-angle model). RK4, dt 1e-4 s, 2 s, sampled every 0.2 s.";

    public static PlanarLink CreateLink() => PlanarLink.SlenderBar("bar", mass: 1.0, length: 1.0);

    public static PendulumTheory CreateTheory()
    {
        var link = CreateLink();
        return new PendulumTheory(link, link.EndA, Vector2d.Zero, PlanarSystem.StandardGravity);
    }

    /// <summary>
    /// The bar pinned to the ground at its end A, at the origin. The bar is
    /// the pin's end A, so the pin reaction is the force on the bar.
    /// </summary>
    public static PlanarSystem CreateSystem(ConstraintStabilization? stabilization = null)
    {
        var link = CreateLink();
        var pivot = new PinJoint("pivot", Attachment.OnLink(link.Id, link.EndA), Attachment.Ground(Vector2d.Zero));
        return new PlanarSystem([link], PlanarSystem.StandardGravity, [pivot], stabilization);
    }

    /// <summary>State with the bar at <paramref name="theta"/>, turning at <paramref name="omega"/>, pin satisfied.</summary>
    public static SystemState CreateState(PlanarSystem system, PendulumTheory theory, double theta, double omega)
    {
        var state = system.CreateState();
        var (pose, twist) = theory.StateAt(theta, omega);
        system.SetBody(state.Values, 0, pose, twist);
        return state;
    }

    public IReadOnlyDictionary<string, double[]> Run()
    {
        var theory = CreateTheory();
        var system = CreateSystem();
        var state = CreateState(system, theory, theory.HangingAngle + Amplitude, 0.0);
        var recorder = new SampleRecorder();

        var initialEnergy = system.KineticEnergy(state.Values) + system.GravitationalPotentialEnergy(state.Values);
        var maxDrift = 0.0;

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, Dt, Steps, (step, s) =>
        {
            var kinetic = system.KineticEnergy(s.Values);
            var potential = system.GravitationalPotentialEnergy(s.Values);
            maxDrift = Math.Max(maxDrift, Math.Abs(kinetic + potential - initialEnergy));

            if (step % Stride != 0)
            {
                return;
            }

            var pose = system.GetPose(s.Values, 0);
            recorder.Add("time", s.Time);
            recorder.Add("x", pose.Position.X);
            recorder.Add("y", pose.Position.Y);
            recorder.Add("theta", pose.Angle);
            recorder.Add("omega", system.GetTwist(s.Values, 0).Angular);
            recorder.Add("kinetic_energy", kinetic);
            recorder.Add("potential_energy", potential);
        });

        recorder.Add("max_energy_drift", maxDrift);
        return recorder.ToOutput();
    }
}
