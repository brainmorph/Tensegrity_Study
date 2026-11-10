namespace Tensegrity.Core.Elements;

/// <summary>
/// Which branch of a <see cref="ContactLaw"/> is active. As with
/// <see cref="CableMode"/>, the force is smooth inside each mode and switches
/// only at events, which the stepper locates and splits the step at
/// (PLAN.md 4.2).
/// </summary>
public enum ContactMode
{
    /// <summary>δ &lt; 0: the surfaces are apart and exert no force.</summary>
    Clear,

    /// <summary>δ ≥ 0 and k·δ + c·δ̇ ≥ 0: the stop pushes back with that force.</summary>
    Pressing,

    /// <summary>
    /// δ ≥ 0 but k·δ + c·δ̇ &lt; 0: still overlapping, but separating so fast
    /// that the damper cancels the spring. A contact cannot pull, so the
    /// force is zero.
    /// </summary>
    Recoiling,
}

/// <summary>
/// A one-sided penalty spring-damper: the compressive counterpart of the
/// cable law. With penetration δ (positive when the surfaces overlap) and
/// its rate δ̇, the force pushing the surfaces apart is
/// <code>
/// F = 0                      if δ &lt; 0     (clear)
/// F = max(0, k·δ + c·δ̇)      if δ ≥ 0     (pressing or recoiling)
/// </code>
/// so it never pulls, however fast the surfaces separate. Used for joint
/// limits (a torsional stop, δ in rad) and the ground line (δ in m).
/// </summary>
public sealed class ContactLaw
{
    public ContactLaw(double stiffness, double damping = 0)
    {
        if (!double.IsFinite(stiffness) || stiffness <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(stiffness), stiffness, "Must be finite and positive.");
        }

        if (!double.IsFinite(damping) || damping < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(damping), damping, "Must be finite and non-negative.");
        }

        Stiffness = stiffness;
        Damping = damping;
    }

    /// <summary>Spring constant k, force per unit penetration.</summary>
    public double Stiffness { get; }

    /// <summary>Viscous damping c, force per unit penetration rate.</summary>
    public double Damping { get; }

    /// <summary>The active branch at penetration δ and rate δ̇.</summary>
    public ContactMode ModeAt(double penetration, double rate)
    {
        if (penetration < 0)
        {
            return ContactMode.Clear;
        }

        return (Stiffness * penetration) + (Damping * rate) >= 0 ? ContactMode.Pressing : ContactMode.Recoiling;
    }

    /// <summary>The physical force at penetration δ and rate δ̇. Never negative.</summary>
    public double Force(double penetration, double rate) =>
        ForceInMode(ModeAt(penetration, rate), penetration, rate);

    /// <summary>
    /// The force formula of a fixed <paramref name="mode"/>, used while the
    /// mode is latched for a step. In <see cref="ContactMode.Pressing"/> it
    /// is the smooth k·δ + c·δ̇ without the clamp, so RK4 stages that stray
    /// just past a switch still see a smooth right-hand side.
    /// </summary>
    public double ForceInMode(ContactMode mode, double penetration, double rate) => mode switch
    {
        ContactMode.Pressing => (Stiffness * penetration) + (Damping * rate),
        ContactMode.Clear or ContactMode.Recoiling => 0.0,
        _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null),
    };

    /// <summary>Energy stored in the spring, ½·k·δ² while overlapping.</summary>
    public double ElasticEnergy(double penetration) =>
        penetration > 0 ? 0.5 * Stiffness * penetration * penetration : 0.0;

    /// <summary>
    /// Power lost in the damper in a fixed mode. Power into the contact is
    /// F·δ̇ and its stored energy changes at k·δ·δ̇, so the damper takes
    /// (F − k·δ)·δ̇: c·δ̇² while pressing, −k·δ·δ̇ while recoiling (positive,
    /// since then δ̇ &lt; −k·δ/c &lt; 0), and nothing while clear.
    /// </summary>
    public double DissipatedPowerInMode(ContactMode mode, double penetration, double rate) => mode switch
    {
        ContactMode.Pressing => Damping * rate * rate,
        ContactMode.Recoiling => -Stiffness * penetration * rate,
        ContactMode.Clear => 0.0,
        _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null),
    };
}
