namespace Tensegrity.Core.Elements;

/// <summary>
/// Which branch of the cable law is active. The tension is smooth inside
/// each mode and switches between them only at events, which the stepper
/// locates and splits the step at (PLAN.md 4.2).
/// </summary>
public enum CableMode
{
    /// <summary>L &lt; L0: the cable is shorter than its rest length and exerts no force.</summary>
    Slack,

    /// <summary>L ≥ L0 and k·(L − L0) + c·L̇ ≥ 0: the cable pulls with that tension.</summary>
    Taut,

    /// <summary>
    /// L ≥ L0 but k·(L − L0) + c·L̇ &lt; 0: still stretched, but shortening so
    /// fast that the damper cancels the spring. A cable cannot push, so the
    /// tension is zero.
    /// </summary>
    Recoiling,
}

/// <summary>
/// An elastic tendon between two attachment points (PLAN.md 4.1). Massless.
/// With stretch Δ = L − L0 and k = EA / L0, the tension is
/// <code>
/// T = 0                          if Δ &lt; 0     (slack)
/// T = max(0, k·Δ + c·L̇)          if Δ ≥ 0     (taut or recoiling)
/// </code>
/// so a slack cable never pulls, however fast it is lengthening, and the
/// damper can lower the tension but never make it compressive.
///
/// With a <see cref="Elements.Muscle"/>, the cable is that muscle's elastic
/// tendon (PLAN.md M9): the contractile part takes up length ℓ of the path,
/// so L and L̇ above are the tendon's own, L_path − ℓ and L̇_path − ℓ̇, and
/// L0, EA and k are the tendon's alone. <see cref="Systems.PlanarSystem"/>
/// does that subtraction; the law here is unchanged.
/// </summary>
public sealed class Cable
{
    public Cable(string id, Attachment a, Attachment b, double restLength, double axialStiffness, double damping = 0, Muscle? muscle = null)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        if (a.IsGround && b.IsGround)
        {
            throw new ArgumentException($"Cable '{id}' joins the ground to itself.");
        }

        if (!a.IsGround && a.LinkId == b.LinkId)
        {
            throw new ArgumentException($"Cable '{id}' has both ends on link '{a.LinkId}', so its length cannot change.");
        }

        RequirePositive(restLength, nameof(restLength));
        RequirePositive(axialStiffness, nameof(axialStiffness));
        if (!double.IsFinite(damping) || damping < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(damping), damping, "Must be finite and non-negative.");
        }

        Id = id;
        A = a;
        B = b;
        RestLength = restLength;
        AxialStiffness = axialStiffness;
        Damping = damping;
        Muscle = muscle;
    }

    public string Id { get; }

    /// <summary>The contractile part in series with this cable, or null for a plain cable.</summary>
    public Muscle? Muscle { get; }

    public Attachment A { get; }

    public Attachment B { get; }

    /// <summary>Rest length L0, m. With a muscle, the tendon's own rest length.</summary>
    public double RestLength { get; }

    /// <summary>Axial stiffness EA, N.</summary>
    public double AxialStiffness { get; }

    /// <summary>Viscous damping coefficient c, N·s/m.</summary>
    public double Damping { get; }

    /// <summary>Spring constant k = EA / L0, N/m.</summary>
    public double Stiffness => AxialStiffness / RestLength;

    /// <summary>Rest length that gives tension <paramref name="tension"/> at length <paramref name="length"/> when at rest: L0 = L·EA / (EA + T).</summary>
    public static double RestLengthFor(double length, double tension, double axialStiffness)
    {
        RequirePositive(length, nameof(length));
        RequirePositive(axialStiffness, nameof(axialStiffness));
        if (!double.IsFinite(tension) || tension < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(tension), tension, "Must be finite and non-negative.");
        }

        return length * axialStiffness / (axialStiffness + tension);
    }

    /// <summary>The active branch of the cable law at length L and rate L̇.</summary>
    public CableMode ModeAt(double length, double lengthRate)
    {
        var stretch = length - RestLength;
        if (stretch < 0)
        {
            return CableMode.Slack;
        }

        return (Stiffness * stretch) + (Damping * lengthRate) >= 0 ? CableMode.Taut : CableMode.Recoiling;
    }

    /// <summary>The physical tension at length L and rate L̇, N. Never negative.</summary>
    public double Tension(double length, double lengthRate) =>
        TensionInMode(ModeAt(length, lengthRate), length, lengthRate);

    /// <summary>
    /// The tension formula of a fixed <paramref name="mode"/>, used while the
    /// mode is latched for a step. In <see cref="CableMode.Taut"/> it is the
    /// smooth k·Δ + c·L̇ without the clamp, so RK4 stages that stray just past
    /// a switch still see a smooth right-hand side.
    /// </summary>
    public double TensionInMode(CableMode mode, double length, double lengthRate) => mode switch
    {
        CableMode.Taut => (Stiffness * (length - RestLength)) + (Damping * lengthRate),
        CableMode.Slack or CableMode.Recoiling => 0.0,
        _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null),
    };

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, value, "Must be finite and positive.");
        }
    }
}
