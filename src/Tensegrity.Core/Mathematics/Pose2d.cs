namespace Tensegrity.Core.Mathematics;

/// <summary>
/// Planar rigid-body pose: the world position of the body origin (its center
/// of mass) and the counter-clockwise orientation angle in radians.
/// </summary>
public readonly record struct Pose2d(Vector2d Position, double Angle)
{
    /// <summary>World position of a body-fixed point.</summary>
    public Vector2d ToWorld(Vector2d localPoint) => Position + localPoint.Rotated(Angle);

    /// <summary>Body-frame coordinates of a world point.</summary>
    public Vector2d ToLocal(Vector2d worldPoint) => (worldPoint - Position).Rotated(-Angle);

    /// <summary>World direction of a body-fixed direction (no translation).</summary>
    public Vector2d RotateToWorld(Vector2d localVector) => localVector.Rotated(Angle);
}
