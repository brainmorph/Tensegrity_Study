using Tensegrity.Core.Bodies;
using Tensegrity.Core.Elements;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;

namespace Tensegrity.Core.Tests.Systems;

/// <summary>
/// Test 1 (PLAN.md section 7) at system level: a cable acts at its true
/// body-fixed attachment points, with the right moments, and obeys
/// Newton's third law.
/// </summary>
public class CableForceTests
{
    private static readonly PlanarLink A = PlanarLink.SlenderBar("a", 1.5, 1.0);
    private static readonly PlanarLink B = PlanarLink.SlenderBar("b", 0.7, 0.6);

    /// <summary>Two free links joined by one undamped cable between off-center points, no gravity.</summary>
    private static (PlanarSystem System, double[] State) TwoLinksAndACable(double restLength = 0.8)
    {
        var cable = new Cable("c", Attachment.OnLink("a", new Vector2d(0.4, 0.05)), Attachment.OnLink("b", new Vector2d(-0.25, -0.1)), restLength, 900.0);
        var system = new PlanarSystem([A, B], Vector2d.Zero, cables: [cable]);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, new Pose2d(new Vector2d(0.1, -0.2), 0.3), new Twist2d(new Vector2d(0.2, 0.1), -0.4));
        system.SetBody(state, 1, new Pose2d(new Vector2d(1.3, 0.5), -1.1), new Twist2d(new Vector2d(-0.3, 0.0), 0.9));
        return (system, state);
    }

    /// <summary>Generalized forces (Fx, Fy, τ per body) from the accelerations Evaluate returns.</summary>
    private static double[] GeneralizedForces(PlanarSystem system, double[] state)
    {
        var dy = new double[system.StateLength];
        system.LatchModes(0, state);
        system.Evaluate(0, state, dy);
        var n = system.CoordinateCount;
        var f = new double[n];
        for (var b = 0; b < system.BodyCount; b++)
        {
            var link = system.Links[b];
            f[3 * b] = dy[n + (3 * b)] * link.Mass;
            f[(3 * b) + 1] = dy[n + (3 * b) + 1] * link.Mass;
            f[(3 * b) + 2] = dy[n + (3 * b) + 2] * link.MomentOfInertia;
        }

        return f;
    }

    private static double ElasticEnergy(PlanarSystem system, double[] state)
    {
        var cable = system.Cables[0];
        var stretch = system.GetCableLength(state, 0).Length - cable.RestLength;
        return stretch > 0 ? 0.5 * cable.Stiffness * stretch * stretch : 0.0;
    }

    [Fact]
    public void UndampedCableForceIsMinusTheGradientOfItsEnergy()
    {
        var (system, state) = TwoLinksAndACable();
        Assert.Equal(CableMode.Taut, system.GetCableMode(state, 0));
        var forces = GeneralizedForces(system, state);
        const double h = 1e-6;

        for (var k = 0; k < system.CoordinateCount; k++)
        {
            var plus = (double[])state.Clone();
            var minus = (double[])state.Clone();
            plus[k] += h;
            minus[k] -= h;
            var gradient = (ElasticEnergy(system, plus) - ElasticEnergy(system, minus)) / (2 * h);

            Assert.Equal(-gradient, forces[k], 1e-6 * Math.Max(1, Math.Abs(gradient)));
        }
    }

    [Fact]
    public void CableForcesConserveLinearAndAngularMomentum()
    {
        var (system, state) = TwoLinksAndACable();
        var forces = GeneralizedForces(system, state);
        var about = new Vector2d(-2, 3);

        var net = new Vector2d(forces[0] + forces[3], forces[1] + forces[4]);
        var moment = 0.0;
        for (var b = 0; b < 2; b++)
        {
            var r = system.GetPose(state, b).Position - about;
            moment += forces[(3 * b) + 2] + Vector2d.Cross(r, new Vector2d(forces[3 * b], forces[(3 * b) + 1]));
        }

        Assert.True(forces.Any(f => Math.Abs(f) > 1), "The cable should be pulling.");
        Assert.Equal(0.0, net.Length, 1e-12);
        Assert.Equal(0.0, moment, 1e-12);
    }

    [Fact]
    public void SlackCableAddsNoForce()
    {
        var (system, state) = TwoLinksAndACable(restLength: 5.0);

        Assert.Equal(CableMode.Slack, system.GetCableMode(state, 0));
        Assert.All(GeneralizedForces(system, state), f => Assert.Equal(0.0, f));
    }

    [Fact]
    public void EvaluateRequiresLatchedModesWhenThereAreCables()
    {
        var (system, state) = TwoLinksAndACable();

        Assert.Throws<InvalidOperationException>(() => system.Evaluate(0, state, new double[system.StateLength]));
    }

    [Fact]
    public void DiagnosticsUseThePhysicalLawNotTheLatch()
    {
        var (system, state) = TwoLinksAndACable();
        var tensions = new double[1];
        system.LatchModes(0, state);

        // Move body b so its end is 0.2 m from a's: the cable is now slack,
        // though the latch says taut.
        var slackState = (double[])state.Clone();
        slackState[3] = 0.9;
        slackState[4] = -0.2;
        system.GetCableTensions(slackState, tensions);

        Assert.Equal(0.0, tensions[0]);
        Assert.False(system.ModesMatch(0, slackState));
        Assert.True(system.ModesMatch(0, state));
    }

    /// <summary>
    /// A horizontal boom of length ℓ pinned at its foot to the origin, held
    /// by a cable from its tip to a ground point at height h above the foot.
    /// Moments about the foot: T·ℓ·h/d = m·g·ℓ/2, so T = m·g·d/(2h) with
    /// d = √(ℓ² + h²). The pin carries the rest: −(T·u + m·g).
    /// </summary>
    [Fact]
    public void CantileverBoomIsInStaticEquilibrium()
    {
        const double length = 1.2;
        const double height = 0.5;
        const double mass = 3.0;
        var boom = PlanarLink.SlenderBar("boom", mass, length);
        var d = Math.Sqrt((length * length) + (height * height));
        var tension = mass * 9.81 * d / (2 * height);
        const double ea = 50_000.0;
        var cable = new Cable("stay", Attachment.OnLink("boom", boom.EndB), Attachment.Ground(new Vector2d(0, height)), Cable.RestLengthFor(d, tension, ea), ea, damping: 5);
        var pin = new PinJoint("foot", Attachment.OnLink("boom", boom.EndA), Attachment.Ground(Vector2d.Zero));
        var system = new PlanarSystem([boom], PlanarSystem.StandardGravity, [pin], [cable]);
        var state = system.CreateState().Values;
        system.SetBody(state, 0, new Pose2d(new Vector2d(length / 2, 0), 0), default);
        var dy = new double[system.StateLength];
        var tensions = new double[1];
        var reactions = new Vector2d[1];

        system.LatchModes(0, state);
        system.Evaluate(0, state, dy);
        system.GetCableTensions(state, tensions);
        system.GetPinReactions(state, reactions);

        var u = new Vector2d(-length, height) / d;
        var expectedReaction = -((tension * u) + (mass * PlanarSystem.StandardGravity));
        Assert.All(dy, a => Assert.Equal(0.0, a, 1e-11));
        Assert.Equal(tension, tensions[0], 1e-9);
        Assert.Equal(expectedReaction.X, reactions[0].X, 1e-9);
        Assert.Equal(expectedReaction.Y, reactions[0].Y, 1e-9);
    }

    [Fact]
    public void RejectsBadCableReferences()
    {
        var cable = new Cable("c", Attachment.OnLink("missing", Vector2d.Zero), Attachment.Ground(Vector2d.Zero), 1, 1);
        var other = new Cable("c", Attachment.OnLink("a", Vector2d.Zero), Attachment.Ground(Vector2d.Zero), 1, 1);

        Assert.Throws<ArgumentException>(() => new PlanarSystem([A], Vector2d.Zero, cables: [cable]));
        Assert.Throws<ArgumentException>(() => new PlanarSystem([A], Vector2d.Zero, cables: [other, other]));
    }

    [Fact]
    public void ZeroLengthCableIsAnError()
    {
        var cable = new Cable("c", Attachment.OnLink("a", Vector2d.Zero), Attachment.Ground(Vector2d.Zero), 1, 1);
        var system = new PlanarSystem([A], Vector2d.Zero, cables: [cable]);

        Assert.Throws<InvalidOperationException>(() => system.LatchModes(0, system.CreateState().Values));
    }
}
