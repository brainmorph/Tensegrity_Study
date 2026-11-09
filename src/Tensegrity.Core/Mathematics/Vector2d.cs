namespace Tensegrity.Core.Mathematics;

/// <summary>
/// Double-precision 2D vector. <see cref="System.Numerics.Vector2"/> is
/// single precision, and all state in the core is float64 (PLAN.md 4.1).
/// </summary>
public readonly record struct Vector2d(double X, double Y)
{
    public static Vector2d Zero => default;

    public static Vector2d UnitX => new(1, 0);

    public static Vector2d UnitY => new(0, 1);

    public double LengthSquared => (X * X) + (Y * Y);

    public double Length => Math.Sqrt(LengthSquared);

    /// <summary>This vector rotated by +90° (counter-clockwise).</summary>
    public Vector2d Perpendicular => new(-Y, X);

    public static Vector2d operator +(Vector2d a, Vector2d b) => new(a.X + b.X, a.Y + b.Y);

    public static Vector2d operator -(Vector2d a, Vector2d b) => new(a.X - b.X, a.Y - b.Y);

    public static Vector2d operator -(Vector2d v) => new(-v.X, -v.Y);

    public static Vector2d operator *(double s, Vector2d v) => new(s * v.X, s * v.Y);

    public static Vector2d operator *(Vector2d v, double s) => new(v.X * s, v.Y * s);

    public static Vector2d operator /(Vector2d v, double s) => new(v.X / s, v.Y / s);

    public static double Dot(Vector2d a, Vector2d b) => (a.X * b.X) + (a.Y * b.Y);

    /// <summary>Scalar 2D cross product: the z component of a × b.</summary>
    public static double Cross(Vector2d a, Vector2d b) => (a.X * b.Y) - (a.Y * b.X);

    /// <summary>ω ẑ × v for a scalar angular rate ω: the velocity of a point at offset v.</summary>
    public static Vector2d Cross(double omega, Vector2d v) => new(-omega * v.Y, omega * v.X);

    /// <summary>This vector rotated counter-clockwise by <paramref name="angle"/> radians.</summary>
    public Vector2d Rotated(double angle)
    {
        var c = Math.Cos(angle);
        var s = Math.Sin(angle);
        return new Vector2d((c * X) - (s * Y), (s * X) + (c * Y));
    }

    public override string ToString() => FormattableString.Invariant($"({X:R}, {Y:R})");
}
