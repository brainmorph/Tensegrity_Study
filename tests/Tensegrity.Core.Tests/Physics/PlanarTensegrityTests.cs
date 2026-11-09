using Tensegrity.Core.Elements;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;
using Tensegrity.Core.Systems;
using Tensegrity.Testing.Scenarios;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Physics;

/// <summary>
/// Test 9 (PLAN.md section 7): the planar two-strut tensegrity holds its
/// prestressed shape at rest with no actuators, carries the closed-form
/// tensions, and returns to its shape after a perturbation. Checked
/// floating with no gravity and standing on pinned feet under gravity.
/// </summary>
public class PlanarTensegrityTests
{
    private static double[] CableLengths(PlanarSystem system, ReadOnlySpan<double> state)
    {
        var lengths = new double[system.Cables.Count];
        for (var c = 0; c < lengths.Length; c++)
        {
            lengths[c] = system.GetCableLength(state, c).Length;
        }

        return lengths;
    }

    private static double[] Tensions(PlanarSystem system, ReadOnlySpan<double> state)
    {
        var tensions = new double[system.Cables.Count];
        system.GetCableTensions(state, tensions);
        return tensions;
    }

    private static double[] Accelerations(PlanarSystem system, ReadOnlySpan<double> state)
    {
        var dy = new double[system.StateLength];
        system.LatchModes(0, state);
        system.Evaluate(0, state, dy);
        return dy[system.CoordinateCount..(2 * system.CoordinateCount)];
    }

    private static double MaxDifference(double[] a, double[] b) => a.Zip(b).Max(p => Math.Abs(p.First - p.Second));

    // ---- Free-floating, no gravity ----

    [Fact]
    public void FreeXCarriesTheSelfStressTensions()
    {
        var model = PlanarX.Free();
        var system = model.System;
        var d = PlanarX.Diagonal;
        var tensions = Tensions(system, model.InitialState);

        // Order: bottom, top, left, right. T_h = C·w/d, T_v = C·h/d.
        var horizontal = PlanarX.StrutCompression * PlanarX.Width / d;
        var vertical = PlanarX.StrutCompression * PlanarX.Height / d;
        Assert.Equal(horizontal, tensions[0], 1e-9);
        Assert.Equal(horizontal, tensions[1], 1e-9);
        Assert.Equal(vertical, tensions[2], 1e-9);
        Assert.Equal(vertical, tensions[3], 1e-9);
        Assert.Equal(PlanarX.Width / PlanarX.Height, tensions[0] / tensions[2], 12);
        Assert.All(Accelerations(system, model.InitialState), a => Assert.Equal(0.0, a, 1e-9));
    }

    [Fact]
    public void FreeXStrutsAreInCompression()
    {
        // At s1's top end (TR) the top cable pulls toward TL and the right
        // cable toward BR. Their resultant along the strut axis, pointing
        // back toward the strut's other end, is the strut compression.
        var model = PlanarX.Free();
        var tensions = Tensions(model.System, model.InitialState);
        var axis = new Vector2d(PlanarX.Width, PlanarX.Height) / PlanarX.Diagonal;
        var atTop = (tensions[1] * new Vector2d(-1, 0)) + (tensions[3] * new Vector2d(0, -1));

        Assert.Equal(PlanarX.StrutCompression, -Vector2d.Dot(atTop, axis), 1e-9);
    }

    [Fact]
    public void FreeXHoldsItsShapeAtRest()
    {
        var model = PlanarX.Free();
        var system = model.System;
        var state = model.CreateInitialState();
        var start = CableLengths(system, state.Values);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000);

        Assert.Equal(0, stepper.EventCount);
        Assert.InRange(MaxDifference(CableLengths(system, state.Values), start), 0.0, 1e-13);
        Assert.InRange(system.KineticEnergy(state.Values), 0.0, 1e-20);
    }

    [Fact]
    public void FreeXReturnsToItsShapeAfterAPerturbation()
    {
        var model = PlanarX.Free();
        var system = model.System;
        var equilibrium = CableLengths(system, model.InitialState);
        var state = PlanarX.PerturbedFree(model);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var maxDeviation = 0.0;
        var minTension = double.MaxValue;
        var maxMomentum = 0.0;

        FixedStepRunner.Run(state, stepper, 1e-4, 30_000, (_, s) =>
        {
            maxDeviation = Math.Max(maxDeviation, MaxDifference(CableLengths(system, s.Values), equilibrium));
            minTension = Math.Min(minTension, Tensions(system, s.Values).Min());
            var linear = (PlanarX.StrutMass * system.GetTwist(s.Values, 0).Linear) + (PlanarX.StrutMass * system.GetTwist(s.Values, 1).Linear);
            maxMomentum = Math.Max(maxMomentum, Math.Max(linear.Length, Math.Abs(system.AngularMomentum(s.Values, Vector2d.Zero))));
        }, stride: 100);

        // The perturbation really changed the shape, no cable went slack,
        // momentum stayed zero, and after 3 s the shape is back.
        Assert.True(maxDeviation > 1e-4, $"Perturbation only moved cables by {maxDeviation} m.");
        Assert.True(minTension > 0, "A cable went slack.");
        Assert.Equal(0, stepper.EventCount);
        Assert.InRange(maxMomentum, 0.0, 1e-12);
        Assert.InRange(MaxDifference(CableLengths(system, state.Values), equilibrium), 0.0, 1e-12);
    }

    // ---- Standing on pinned feet, under gravity ----

    [Fact]
    public void PinnedXCarriesTheStaticTensionsAndReactions()
    {
        var model = PlanarX.Pinned();
        var system = model.System;
        var weight = PlanarX.StrutMass * 9.81;
        var tensions = Tensions(system, model.InitialState);
        var reactions = new Vector2d[2];
        system.GetPinReactions(model.InitialState, reactions);

        // Moments about each foot: T_top·h = w·(T_side + m·g/2).
        var top = PlanarX.Width / PlanarX.Height * (PlanarX.SideCableTension + (weight / 2));
        Assert.Equal(top, tensions[0], 1e-9);
        Assert.Equal(PlanarX.SideCableTension, tensions[1], 1e-9);
        Assert.Equal(PlanarX.SideCableTension, tensions[2], 1e-9);

        // Force balance on each strut: the side cables cancel along it, so
        // the foot pushes out against the top cable and carries the weight.
        Assert.Equal(top, reactions[0].X, 1e-9);
        Assert.Equal(weight, reactions[0].Y, 1e-9);
        Assert.Equal(-top, reactions[1].X, 1e-9);
        Assert.Equal(weight, reactions[1].Y, 1e-9);
        Assert.All(Accelerations(system, model.InitialState), a => Assert.Equal(0.0, a, 1e-9));
    }

    [Fact]
    public void PinnedXHoldsStillWithNoActuators()
    {
        var model = PlanarX.Pinned();
        var system = model.System;
        var state = model.CreateInitialState();
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);

        FixedStepRunner.Run(state, stepper, 1e-4, 20_000);

        for (var b = 0; b < 2; b++)
        {
            var start = system.GetPose(model.InitialState, b);
            var end = system.GetPose(state.Values, b);
            Assert.InRange((end.Position - start.Position).Length, 0.0, 1e-13);
            Assert.InRange(Math.Abs(end.Angle - start.Angle), 0.0, 1e-13);
        }

        Assert.Equal(0, stepper.EventCount);
        Assert.InRange(system.KineticEnergy(state.Values), 0.0, 1e-20);
    }

    [Fact]
    public void PinnedXReturnsToItsPoseAfterAKick()
    {
        var model = PlanarX.Pinned();
        var system = model.System;
        var state = PlanarX.KickedPinned(model);
        var stepper = new EventSplittingStepper(new Rk4Integrator(), system);
        var maxSwing = 0.0;
        var minTension = double.MaxValue;

        FixedStepRunner.Run(state, stepper, 1e-4, 60_000, (_, s) =>
        {
            maxSwing = Math.Max(maxSwing, Math.Abs(system.GetPose(s.Values, 0).Angle - system.GetPose(model.InitialState, 0).Angle));
            minTension = Math.Min(minTension, Tensions(system, s.Values).Min());
        }, stride: 100);

        Assert.True(maxSwing > 1e-4, $"The kick only turned the strut by {maxSwing} rad.");
        Assert.True(minTension > 0, "A cable went slack.");
        Assert.Equal(0, stepper.EventCount);
        for (var b = 0; b < 2; b++)
        {
            var start = system.GetPose(model.InitialState, b);
            var end = system.GetPose(state.Values, b);
            Assert.InRange((end.Position - start.Position).Length, 0.0, 1e-8);
            Assert.InRange(Math.Abs(end.Angle - start.Angle), 0.0, 1e-8);
        }
    }

    [Fact]
    public void GeneratorRejectsBadParameters()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => PlanarTensegrityX.FreeFloating(0, 1, 1, 1, 0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => PlanarTensegrityX.FreeFloating(1, 1, 1, 1, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => PlanarTensegrityX.PinnedFeet(1, 1, 1, 1, 0, 1, new Vector2d(1, -9.81)));
        Assert.Throws<ArgumentOutOfRangeException>(() => PlanarTensegrityX.PinnedFeet(1, 1, 1, 1, 0, -1, PlanarSystem.StandardGravity));
    }

    [Fact]
    public void GeneratedGeometryPutsStrutEndsOnTheRectangleCorners()
    {
        var model = PlanarX.Pinned();
        var system = model.System;
        var s1 = system.Links[0];
        var s2 = system.Links[1];
        Vector2d End(string id, Vector2d local) => system.WorldPoint(model.InitialState, Attachment.OnLink(id, local));
        const double w = PlanarX.Width;
        const double h = PlanarX.Height;

        Assert.Equal(0.0, (End("s1", s1.EndA) - new Vector2d(-w / 2, 0)).Length, 15);
        Assert.Equal(0.0, (End("s1", s1.EndB) - new Vector2d(w / 2, h)).Length, 15);
        Assert.Equal(0.0, (End("s2", s2.EndA) - new Vector2d(w / 2, 0)).Length, 15);
        Assert.Equal(0.0, (End("s2", s2.EndB) - new Vector2d(-w / 2, h)).Length, 15);
        Assert.Equal(0.0, system.PinPositionResidual(model.InitialState), 15);
    }
}
