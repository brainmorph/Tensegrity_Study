using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Bodies;

/// <summary>
/// A rigid link (strut) in the plane: three degrees of freedom (x, y, θ).
/// The body frame has its origin at the center of mass with the link axis
/// along local +x, so the ends are at (∓Length/2, 0).
/// </summary>
public sealed class PlanarLink
{
    public PlanarLink(string id, double mass, double momentOfInertia, double length)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        RequirePositive(mass, nameof(mass));
        RequirePositive(momentOfInertia, nameof(momentOfInertia));
        if (!double.IsFinite(length) || length < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(length), length, "Must be finite and non-negative.");
        }

        Id = id;
        Mass = mass;
        MomentOfInertia = momentOfInertia;
        Length = length;
    }

    public string Id { get; }

    /// <summary>Mass in kg.</summary>
    public double Mass { get; }

    /// <summary>Moment of inertia about the center of mass, kg·m².</summary>
    public double MomentOfInertia { get; }

    /// <summary>Length in m.</summary>
    public double Length { get; }

    /// <summary>Body-frame position of the end at local −x.</summary>
    public Vector2d EndA => new(-Length / 2, 0);

    /// <summary>Body-frame position of the end at local +x.</summary>
    public Vector2d EndB => new(Length / 2, 0);

    /// <summary>A uniform slender bar: I = m·ℓ²/12.</summary>
    public static PlanarLink SlenderBar(string id, double mass, double length)
    {
        RequirePositive(length, nameof(length));
        return new PlanarLink(id, mass, mass * length * length / 12, length);
    }

    /// <summary>Moment of inertia about a body-fixed point (parallel-axis theorem).</summary>
    public double MomentOfInertiaAbout(Vector2d localPoint) => MomentOfInertia + (Mass * localPoint.LengthSquared);

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }
}
