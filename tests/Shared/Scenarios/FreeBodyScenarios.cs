using Tensegrity.Core.Bodies;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Testing.Scenarios;

/// <summary>
/// Test 2 (PLAN.md section 7): a free link thrown under gravity. The motion is
/// quadratic in time, which RK4 integrates exactly up to rounding.
/// </summary>
public sealed class FreeFallScenario : IReferenceScenario
{
    public const double Dt = 1e-3;
    public const long Steps = 2000;
    public const long Stride = 200;

    public static readonly Pose2d InitialPose = new(new Vector2d(0, 10), 0.3);
    public static readonly Twist2d InitialTwist = new(new Vector2d(1.5, 2.0), 0);

    public string Name => "free_fall";

    public string Description =>
        "Slender bar (2 kg, 0.5 m) thrown from (0, 10) m at (1.5, 2.0) m/s, no spin, standard gravity. RK4, dt 1e-3 s, 2 s, sampled every 0.2 s.";

    public static PlanarSystem CreateSystem() =>
        new([PlanarLink.SlenderBar("bar", mass: 2.0, length: 0.5)], PlanarSystem.StandardGravity);

    public IReadOnlyDictionary<string, double[]> Run() =>
        FreeBodyRecording.Record(CreateSystem(), InitialPose, InitialTwist, Dt, Steps, Stride, includeAngularMomentum: false);
}

/// <summary>
/// Test 2 (PLAN.md section 7): a free link spinning with no gravity. Kinetic
/// energy and angular momentum are conserved; θ grows linearly.
/// </summary>
public sealed class SpinningBodyScenario : IReferenceScenario
{
    public const double Dt = 1e-3;
    public const long Steps = 2000;
    public const long Stride = 200;

    public static readonly Pose2d InitialPose = new(new Vector2d(1, 2), 0);
    public static readonly Twist2d InitialTwist = new(new Vector2d(0.4, -0.3), 3.0);

    public string Name => "spinning_body";

    public string Description =>
        "Slender bar (1.5 kg, 0.8 m) at (1, 2) m moving at (0.4, -0.3) m/s and spinning at 3 rad/s, no gravity. RK4, dt 1e-3 s, 2 s, sampled every 0.2 s.";

    public static PlanarSystem CreateSystem() =>
        new([PlanarLink.SlenderBar("bar", mass: 1.5, length: 0.8)], Vector2d.Zero);

    public IReadOnlyDictionary<string, double[]> Run() =>
        FreeBodyRecording.Record(CreateSystem(), InitialPose, InitialTwist, Dt, Steps, Stride, includeAngularMomentum: true);
}

internal static class FreeBodyRecording
{
    public static IReadOnlyDictionary<string, double[]> Record(
        PlanarSystem system, Pose2d pose, Twist2d twist, double dt, long steps, long stride, bool includeAngularMomentum)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, pose, twist);
        var recorder = new SampleRecorder();

        FixedStepRunner.Run(state, new Rk4Integrator(), system.Evaluate, dt, steps, (_, s) =>
        {
            var p = system.GetPose(s.Values, 0);
            recorder.Add("time", s.Time);
            recorder.Add("x", p.Position.X);
            recorder.Add("y", p.Position.Y);
            recorder.Add("theta", p.Angle);
            recorder.Add("kinetic_energy", system.KineticEnergy(s.Values));
            if (includeAngularMomentum)
            {
                recorder.Add("angular_momentum", system.AngularMomentum(s.Values, Vector2d.Zero));
            }
            else
            {
                recorder.Add("potential_energy", system.GravitationalPotentialEnergy(s.Values));
            }
        }, stride);

        return recorder.ToOutput();
    }
}
