using Tensegrity.Core.Bodies;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// The midpoint axial force (compression positive) used to flag struts in
/// tension (PLAN.md 4.1), checked against hand-derived closed forms that
/// include the link's own weight and inertia.
/// </summary>
public class AxialForceTests
{
    private static double[] Compressions(PlanarSystem system, ReadOnlySpan<double> state)
    {
        var values = new double[system.BodyCount];
        system.GetMidpointCompressions(0, state, values);
        return values;
    }

    [Fact]
    public void FreeXStrutsCarryThePrestressCompression()
    {
        var model = PlanarX.Free();

        var compressions = Compressions(model.System, model.InitialState);

        Assert.All(model.System.Links, l => Assert.Equal(LinkRole.Strut, l.Role));
        Assert.Equal(PlanarX.StrutCompression, compressions[0], 1e-9);
        Assert.Equal(PlanarX.StrutCompression, compressions[1], 1e-9);
    }

    [Fact]
    public void PinnedXStrutsCarryCableLoadsPlusHalfTheirWeight()
    {
        // The +x half of s1 holds its top end: the top cable pulls it toward
        // TL and the right cable toward BR. With half the weight, the section
        // force is (T_top, T_side + m·g/2); along the axis (w, h)/d that is
        // (T_top·w + (T_side + m·g/2)·h) / d. s2 is the mirror image.
        var model = PlanarX.Pinned();
        var weight = PlanarX.StrutMass * 9.81;
        var top = PlanarX.Width / PlanarX.Height * (PlanarX.SideCableTension + (weight / 2));
        var expected = ((top * PlanarX.Width) + ((PlanarX.SideCableTension + (weight / 2)) * PlanarX.Height)) / PlanarX.Diagonal;

        var compressions = Compressions(model.System, model.InitialState);

        Assert.Equal(expected, compressions[0], 1e-9);
        Assert.Equal(expected, compressions[1], 1e-9);
    }

    [Fact]
    public void HangingBarIsInTensionFromTheWeightOfItsLowerHalf()
    {
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle, 0.0);

        Assert.Equal(-0.5 * 9.81, Compressions(system, state.Values)[0], 1e-12);
    }

    [Fact]
    public void SwingingBarAddsTheCentripetalLoadOfItsLowerHalf()
    {
        // At the bottom of the swing the lower half (center 3ℓ/4 from the
        // pivot) needs (m/2)·ω²·3ℓ/4 toward the pivot on top of its weight.
        const double omega = 2.5;
        var theory = PendulumScenario.CreateTheory();
        var system = PendulumScenario.CreateSystem();
        var state = PendulumScenario.CreateState(system, theory, theory.HangingAngle, omega);

        Assert.Equal(-0.5 * (9.81 + (omega * omega * 0.75)), Compressions(system, state.Values)[0], 1e-12);
    }

    [Fact]
    public void SpinningRodIsInCentripetalTensionAtItsMiddle()
    {
        // Classic result: tension m·ω²·ℓ/8 at the center of a spinning rod.
        const double omega = 4.0;
        var rod = PlanarLink.SlenderBar("rod", 1.2, 0.8);
        var system = new PlanarSystem([rod], Vector2d.Zero);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(new Vector2d(1, 2), 0.7), new Twist2d(Vector2d.Zero, omega));

        Assert.Equal(-1.2 * omega * omega * 0.8 / 8, Compressions(system, state.Values)[0], 1e-12);
    }

    [Fact]
    public void FreeFallingBarCarriesNoAxialForce()
    {
        var bar = PlanarLink.SlenderBar("bar", 2, 1);
        var system = new PlanarSystem([bar], PlanarSystem.StandardGravity);
        var state = system.CreateState();
        system.SetBody(state.Values, 0, new Pose2d(Vector2d.Zero, 1.1), default);

        Assert.Equal(0.0, Compressions(system, state.Values)[0], 1e-14);
    }
}
