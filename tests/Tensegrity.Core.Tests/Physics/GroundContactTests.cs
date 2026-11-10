using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// The ground line in motion (PLAN.md 4.1): a 1 kg, 1 m bar with a contact
/// point at each end. Checked against closed forms for resting, sliding
/// and an incline (gravity tilted by α), then the energy balance and RK4
/// order through a drop that visits every mode.
/// </summary>
public class GroundContactTests
{
    private const double Kn = 1e4;
    private const double Cn = 60.0;
    private const double Kt = 1e4;
    private const double Ct = 60.0;
    private const double Mu = 0.5;
    private const double G = 9.81;

    private static readonly PlanarLink Bar = PlanarLink.SlenderBar("bar", mass: 1.0, length: 1.0);

    private static PlanarSystem BarOnGround(Vector2d gravity, double friction = Mu) => new(
        [Bar],
        gravity,
        ground: new GroundLine(
            [new ContactPoint("left", "bar", Bar.EndA), new ContactPoint("right", "bar", Bar.EndB)],
            Kn,
            Cn,
            friction,
            Kt,
            Ct));

    /// <summary>Gravity of magnitude g tilted so the ground acts as an incline rising to −x at angle α.</summary>
    private static Vector2d Incline(double alpha) => new(G * Math.Sin(alpha), -G * Math.Cos(alpha));

    /// <summary>The bar lying flat with each end at its static penetration under gravity normal g_n.</summary>
    private static SystemState Lying(PlanarSystem system, double normalGravity, double vx = 0)
    {
        var state = system.CreateState();
        var sink = Bar.Mass * normalGravity / (2 * Kn);
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, -sink), 0), new Twist2d(new Vector2d(vx, 0), 0));
        return state;
    }

    private static EventSplittingStepper Stepper(PlanarSystem system) => new(new Rk4Integrator(), system);

    [Fact]
    public void BarRestsInExactEquilibriumAndSticks()
    {
        var system = BarOnGround(PlanarSystem.StandardGravity);
        var state = Lying(system, G);
        var start = system.GetPose(state.Values, 0);

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 10_000);

        var pose = system.GetPose(state.Values, 0);
        Assert.Equal(start.Position.X, pose.Position.X, 13);
        Assert.Equal(start.Position.Y, pose.Position.Y, 13);
        Assert.Equal(0.0, pose.Angle, 13);
        Assert.Equal((ContactMode.Pressing, FrictionMode.Stick), system.GetContactModes(state.Values, 0));

        var forces = new Vector2d[2];
        system.GetContactForces(state.Values, forces);
        Assert.Equal(Bar.Mass * G / 2, forces[0].Y, 10);
        Assert.Equal(0.0, forces[0].X, 10);
    }

    [Fact]
    public void SlidingBarDeceleratesAtExactlyMuG()
    {
        const double v0 = 1.0;
        var system = BarOnGround(PlanarSystem.StandardGravity);
        var state = Lying(system, G, v0);
        var energy = system.TotalEnergy(state.Values);
        var stepper = Stepper(system);
        var y0 = system.GetPose(state.Values, 0).Position.Y;

        // Friction acts along the bar's axis, through its center: no torque, no change in load.
        FixedStepRunner.Run(state, stepper, 1e-4, 1_000);
        Assert.Equal((ContactMode.Pressing, FrictionMode.SlipPositive), system.GetContactModes(state.Values, 1));
        Assert.Equal(v0 - (Mu * G * 0.1), system.GetTwist(state.Values, 0).Linear.X, 11);
        Assert.Equal(y0, system.GetPose(state.Values, 0).Position.Y, 13);

        FixedStepRunner.Run(state, stepper, 1e-4, 9_000);

        // It stops after v0²/(2μg), less the few tenths of a mm the tangential springs gave.
        Assert.Equal(FrictionMode.Stick, system.GetContactModes(state.Values, 0).Friction);
        Assert.InRange(Math.Abs(system.GetTwist(state.Values, 0).Linear.X), 0.0, 1e-9);
        Assert.Equal(v0 * v0 / (2 * Mu * G), system.GetPose(state.Values, 0).Position.X, 3);
        Assert.Equal(0.5 * Bar.Mass * v0 * v0, system.DissipatedEnergy(state.Values), 6);
        Assert.Equal(energy, system.TotalEnergy(state.Values), 10);
    }

    [Fact]
    public void BarOnAnInclineBelowTheFrictionAngleSticksWithoutCreep()
    {
        var alpha = 20 * Math.PI / 180;
        Assert.True(Math.Tan(alpha) < Mu);
        var system = BarOnGround(Incline(alpha));
        var state = Lying(system, G * Math.Cos(alpha));
        var stepper = Stepper(system);

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000);
        var settled = system.GetPose(state.Values, 0).Position.X;
        FixedStepRunner.Run(state, stepper, 1e-4, 10_000);

        // Each tangential spring holds half the downhill weight.
        Assert.Equal(Bar.Mass * G * Math.Sin(alpha) / (2 * Kt), state.Values[system.ExtensionIndex(0)], 12);
        Assert.Equal(settled, system.GetPose(state.Values, 0).Position.X, 14);
        Assert.Equal((ContactMode.Pressing, FrictionMode.Stick), system.GetContactModes(state.Values, 1));
    }

    [Fact]
    public void BarOnAnInclineAboveTheFrictionAngleSlidesAtTheCoulombRate()
    {
        var alpha = 35 * Math.PI / 180;
        Assert.True(Math.Tan(alpha) > Mu);
        var system = BarOnGround(Incline(alpha));
        var state = Lying(system, G * Math.Cos(alpha));
        var stepper = Stepper(system);

        FixedStepRunner.Run(state, stepper, 1e-4, 5_000);
        var v1 = system.GetTwist(state.Values, 0).Linear.X;
        FixedStepRunner.Run(state, stepper, 1e-4, 5_000);
        var v2 = system.GetTwist(state.Values, 0).Linear.X;

        Assert.Equal(FrictionMode.SlipPositive, system.GetContactModes(state.Values, 0).Friction);
        Assert.Equal(G * (Math.Sin(alpha) - (Mu * Math.Cos(alpha))) * 0.5, v2 - v1, 10);
    }

    /// <summary>The bar released tilted 30° with its center 0.4 m up, moving sideways: it lands, bounces, slides and stops.</summary>
    private static SystemState Dropped(PlanarSystem system)
    {
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(0, 0.4), Math.PI / 6), new Twist2d(new Vector2d(1.5, 0), -2.0));
        return state;
    }

    [Fact]
    public void DroppedBarEnergyBalanceClosesThroughEveryMode()
    {
        var system = BarOnGround(PlanarSystem.StandardGravity);
        var state = Dropped(system);
        var initial = system.TotalEnergy(state.Values);
        var normals = new HashSet<ContactMode>();
        var frictions = new HashSet<FrictionMode>();
        var drift = 0.0;
        var previous = 0.0;
        var monotone = true;
        var stepper = Stepper(system);

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000, (_, s) =>
        {
            for (var i = 0; i < system.ContactCount; i++)
            {
                var (normal, friction) = system.GetContactModes(s.Values, i);
                normals.Add(normal);
                frictions.Add(friction);
            }

            drift = Math.Max(drift, Math.Abs(system.TotalEnergy(s.Values) - initial));
            var dissipated = system.DissipatedEnergy(s.Values);
            monotone &= dissipated >= previous;
            previous = dissipated;
        });

        Assert.Equal([ContactMode.Clear, ContactMode.Pressing, ContactMode.Recoiling], normals.Order());
        Assert.Equal([FrictionMode.Free, FrictionMode.Stick, FrictionMode.SlipPositive, FrictionMode.SlipNegative], frictions.Order());
        Assert.True(monotone, "Dissipated energy must never decrease.");
        Assert.Equal(0, stepper.UnresolvedSteps);

        // Measured 2.7e-8 J against 5.2 J dissipated, worst while sliding
        // near 0.7 s. It is integration error: 6.7e-6, 4.3e-7, 2.7e-8,
        // 1.7e-9, 1.1e-10 J from dt = 4e-4 down, 15.4-16.0x per halving,
        // with the same 22 events at every step size.
        Assert.InRange(drift, 0.0, 5e-8);

        // At rest on the ground, lying flat.
        Assert.InRange(system.KineticEnergy(state.Values), 0.0, 1e-12);
        Assert.Equal(0.0, Math.Sin(system.GetPose(state.Values, 0).Angle), 6);
    }

    [Fact]
    public void Rk4KeepsFourthOrderThroughContactEvents()
    {
        static Pose2d FinalPose(double dt)
        {
            var system = BarOnGround(PlanarSystem.StandardGravity);
            var state = Dropped(system);
            FixedStepRunner.Run(state, Stepper(system), dt, (long)Math.Round(0.6 / dt));
            return system.GetPose(state.Values, 0);
        }

        var reference = FinalPose(2.5e-5);
        double Error(double dt)
        {
            var pose = FinalPose(dt);
            return Math.Max((pose.Position - reference.Position).Length, Math.Abs(pose.Angle - reference.Angle));
        }

        var errors = new[] { 4e-4, 2e-4, 1e-4 }.Select(Error).ToArray();
        Assert.InRange(errors[0] / errors[1], 12.0, 20.0);
        Assert.InRange(errors[1] / errors[2], 12.0, 20.0);
    }

    [Fact]
    public void FrictionlessGroundLetsTheBarSlideForever()
    {
        var system = BarOnGround(PlanarSystem.StandardGravity, friction: 0);
        var state = Lying(system, G, 0.7);

        FixedStepRunner.Run(state, Stepper(system), 1e-4, 5_000);

        // The tangential springs relax at once; after that nothing slows the bar.
        Assert.Equal(0.7, system.GetTwist(state.Values, 0).Linear.X, 3);
        Assert.Equal(FrictionMode.SlipPositive, system.GetContactModes(state.Values, 0).Friction);
    }

    [Fact]
    public void StateCarriesOneExtensionPerContactPoint()
    {
        var system = BarOnGround(PlanarSystem.StandardGravity);

        Assert.Equal(2, system.ContactCount);
        Assert.Equal((6 * 1) + 2 + 2, system.StateLength);
        Assert.Equal(8, system.ExtensionIndex(0));
        Assert.Equal(9, system.ExtensionIndex(1));
        Assert.Throws<ArgumentOutOfRangeException>(() => system.ExtensionIndex(2));
        Assert.Throws<ArgumentException>(() => new PlanarSystem(
            [Bar],
            Vector2d.Zero,
            ground: new GroundLine([new ContactPoint("toe", "foot", Vector2d.Zero)], Kn, Cn, Mu, Kt, Ct)));
    }
}
