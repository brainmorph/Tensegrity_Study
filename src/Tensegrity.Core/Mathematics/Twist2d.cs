namespace Tensegrity.Core.Mathematics;

/// <summary>
/// Planar rigid-body velocity: center-of-mass linear velocity and angular
/// rate (rad/s, counter-clockwise positive).
/// </summary>
public readonly record struct Twist2d(Vector2d Linear, double Angular)
{
    /// <summary>World velocity of a body-fixed point at world offset <paramref name="offsetFromCenter"/>.</summary>
    public Vector2d VelocityOfPoint(Vector2d offsetFromCenter) =>
        Linear + Vector2d.Cross(Angular, offsetFromCenter);
}
