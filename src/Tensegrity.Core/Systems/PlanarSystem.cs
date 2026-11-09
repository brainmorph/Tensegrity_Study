using Tensegrity.Core.Bodies;
using Tensegrity.Core.Integration;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Systems;

/// <summary>
/// A set of planar rigid links under uniform gravity, in maximal coordinates.
///
/// State layout for n bodies (length 6n), positions first, then velocities:
/// <code>
/// [ x0 y0 θ0  x1 y1 θ1 … | ẋ0 ẏ0 θ̇0  ẋ1 ẏ1 θ̇1 … ]
/// </code>
/// Splitting positions from velocities keeps the generalized-coordinate block
/// contiguous for the constraint solver in M2.
///
/// M1 scope: gravity only. Pins, cables and actuators arrive in M2+.
/// </summary>
public sealed class PlanarSystem
{
    /// <summary>Standard gravity used throughout the study (PLAN.md 4.1), m/s².</summary>
    public static readonly Vector2d StandardGravity = new(0, -9.81);

    public const int DofPerBody = 3;

    private readonly PlanarLink[] links;

    public PlanarSystem(IEnumerable<PlanarLink> links, Vector2d gravity)
    {
        ArgumentNullException.ThrowIfNull(links);
        this.links = links.ToArray();
        if (this.links.Length == 0)
        {
            throw new ArgumentException("A system needs at least one link.", nameof(links));
        }

        var duplicate = this.links.GroupBy(l => l.Id, StringComparer.Ordinal).FirstOrDefault(g => g.Count() > 1);
        if (duplicate is not null)
        {
            throw new ArgumentException($"Duplicate link id '{duplicate.Key}'.", nameof(links));
        }

        if (!double.IsFinite(gravity.X) || !double.IsFinite(gravity.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(gravity), gravity, "Must be finite.");
        }

        Gravity = gravity;
    }

    public IReadOnlyList<PlanarLink> Links => links;

    public Vector2d Gravity { get; }

    public int BodyCount => links.Length;

    /// <summary>Number of generalized coordinates: 3 per body.</summary>
    public int CoordinateCount => DofPerBody * links.Length;

    /// <summary>Length of the state vector: coordinates plus velocities.</summary>
    public int StateLength => 2 * CoordinateCount;

    public int PositionIndex(int body) => DofPerBody * CheckBody(body);

    public int VelocityIndex(int body) => CoordinateCount + (DofPerBody * CheckBody(body));

    public SystemState CreateState(double time = 0) => new(StateLength, time);

    public Pose2d GetPose(ReadOnlySpan<double> state, int body)
    {
        CheckLength(state.Length);
        var i = PositionIndex(body);
        return new Pose2d(new Vector2d(state[i], state[i + 1]), state[i + 2]);
    }

    public Twist2d GetTwist(ReadOnlySpan<double> state, int body)
    {
        CheckLength(state.Length);
        var i = VelocityIndex(body);
        return new Twist2d(new Vector2d(state[i], state[i + 1]), state[i + 2]);
    }

    public void SetBody(Span<double> state, int body, Pose2d pose, Twist2d twist)
    {
        CheckLength(state.Length);
        var p = PositionIndex(body);
        state[p] = pose.Position.X;
        state[p + 1] = pose.Position.Y;
        state[p + 2] = pose.Angle;
        var v = VelocityIndex(body);
        state[v] = twist.Linear.X;
        state[v + 1] = twist.Linear.Y;
        state[v + 2] = twist.Angular;
    }

    /// <summary>
    /// dy/dt for the whole system. Forces and torques are accumulated per
    /// body and then divided by mass and inertia, so later force elements
    /// slot in without changing the structure.
    /// </summary>
    public void Evaluate(double time, ReadOnlySpan<double> state, Span<double> derivative)
    {
        CheckLength(state.Length);
        CheckLength(derivative.Length);
        var n = CoordinateCount;

        // dq/dt = v
        state[n..].CopyTo(derivative[..n]);

        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var force = link.Mass * Gravity;
            const double torque = 0.0; // Uniform gravity acts at the center of mass.

            var a = n + (DofPerBody * b);
            derivative[a] = force.X / link.Mass;
            derivative[a + 1] = force.Y / link.Mass;
            derivative[a + 2] = torque / link.MomentOfInertia;
        }
    }

    /// <summary>Translational plus rotational kinetic energy, J.</summary>
    public double KineticEnergy(ReadOnlySpan<double> state)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var twist = GetTwist(state, b);
            total += (0.5 * link.Mass * twist.Linear.LengthSquared)
                + (0.5 * link.MomentOfInertia * twist.Angular * twist.Angular);
        }

        return total;
    }

    /// <summary>Gravitational potential energy, −Σ m·(g·r), zero at the origin, J.</summary>
    public double GravitationalPotentialEnergy(ReadOnlySpan<double> state)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            total -= links[b].Mass * Vector2d.Dot(Gravity, GetPose(state, b).Position);
        }

        return total;
    }

    /// <summary>Total angular momentum about a fixed world point, kg·m²/s.</summary>
    public double AngularMomentum(ReadOnlySpan<double> state, Vector2d about)
    {
        var total = 0.0;
        for (var b = 0; b < links.Length; b++)
        {
            var link = links[b];
            var twist = GetTwist(state, b);
            var r = GetPose(state, b).Position - about;
            total += (link.MomentOfInertia * twist.Angular) + (link.Mass * Vector2d.Cross(r, twist.Linear));
        }

        return total;
    }

    private int CheckBody(int body)
    {
        if ((uint)body >= (uint)links.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(body), body, $"System has {links.Length} bodies.");
        }

        return body;
    }

    private void CheckLength(int length)
    {
        if (length != StateLength)
        {
            throw new ArgumentException($"State length {length} does not match system state length {StateLength}.");
        }
    }
}
