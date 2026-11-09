using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Mathematics;

/// <summary>Test 1 (part): vector algebra and body-frame ↔ world-frame transforms.</summary>
public class GeometryTests
{
    private const double Eps = 1e-14;

    [Fact]
    public void ScalarCrossIsZComponentOfCrossProduct()
    {
        Assert.Equal(1.0, Vector2d.Cross(Vector2d.UnitX, Vector2d.UnitY));
        Assert.Equal(-1.0, Vector2d.Cross(Vector2d.UnitY, Vector2d.UnitX));
        Assert.Equal(3.0 * 5.0 - 4.0 * 2.0, Vector2d.Cross(new Vector2d(3, 4), new Vector2d(2, 5)));
    }

    [Fact]
    public void AngularCrossGivesPointVelocity()
    {
        // ω ẑ × (1, 0) = (0, ω): a point on +x moves toward +y for positive spin.
        Assert.Equal(new Vector2d(0, 2), Vector2d.Cross(2.0, Vector2d.UnitX));
        Assert.Equal(new Vector2d(-2, 0), Vector2d.Cross(2.0, Vector2d.UnitY));
    }

    [Fact]
    public void RotationByQuarterTurnIsCounterClockwise()
    {
        var r = Vector2d.UnitX.Rotated(Math.PI / 2);

        Assert.Equal(0.0, r.X, Eps);
        Assert.Equal(1.0, r.Y, Eps);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(0.7)]
    [InlineData(-2.9)]
    [InlineData(123.4)]
    public void RotationPreservesLengthAndDot(double angle)
    {
        var a = new Vector2d(3, -4);
        var b = new Vector2d(-1.5, 0.25);

        Assert.Equal(a.Length, a.Rotated(angle).Length, Eps * 10);
        Assert.Equal(Vector2d.Dot(a, b), Vector2d.Dot(a.Rotated(angle), b.Rotated(angle)), Eps * 10);
        Assert.Equal(Vector2d.Cross(a, b), Vector2d.Cross(a.Rotated(angle), b.Rotated(angle)), Eps * 10);
    }

    [Fact]
    public void PoseMapsBodyPointToWorld()
    {
        var pose = new Pose2d(new Vector2d(1, 2), Math.PI / 2);

        var world = pose.ToWorld(new Vector2d(0.5, 0));

        Assert.Equal(1.0, world.X, Eps);
        Assert.Equal(2.5, world.Y, Eps);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(1.1)]
    [InlineData(-3.0)]
    public void PoseWorldLocalRoundTrip(double angle)
    {
        var pose = new Pose2d(new Vector2d(-0.3, 4.2), angle);
        var local = new Vector2d(0.75, -0.2);

        var back = pose.ToLocal(pose.ToWorld(local));

        Assert.Equal(local.X, back.X, Eps);
        Assert.Equal(local.Y, back.Y, Eps);
    }

    [Fact]
    public void TwistGivesRigidPointVelocity()
    {
        var twist = new Twist2d(new Vector2d(1, 0), 2.0);

        Assert.Equal(new Vector2d(1, 1), twist.VelocityOfPoint(new Vector2d(0.5, 0)));
    }
}
