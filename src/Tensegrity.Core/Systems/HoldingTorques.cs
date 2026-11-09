namespace Tensegrity.Core.Systems;

/// <summary>
/// The effort needed to hold a state still (PLAN.md 7, test 11): the joint
/// torques that make every acceleration zero, from
/// <see cref="PlanarSystem.SolveHoldingTorques"/>.
/// </summary>
/// <param name="Pins">The actuated pins, in the order asked for.</param>
/// <param name="Torques">
/// Torque at each pin, N·m, with the actuator sign convention: +τ on the
/// pin's end A, −τ on end B. Zero means the structure holds that joint by
/// itself.
/// </param>
/// <param name="MaxLinearAcceleration">
/// Largest center-of-mass acceleration left with these torques applied,
/// m/s². Zero (to round-off) when the pins can hold the state; positive
/// when they cannot, for example when a joint that needs effort is not in
/// <paramref name="Pins"/>.
/// </param>
/// <param name="MaxAngularAcceleration">Largest angular acceleration left, rad/s².</param>
public sealed record HoldingTorques(
    IReadOnlyList<string> Pins,
    IReadOnlyList<double> Torques,
    double MaxLinearAcceleration,
    double MaxAngularAcceleration)
{
    /// <summary>Torque at pin <paramref name="pinId"/>, N·m.</summary>
    public double this[string pinId]
    {
        get
        {
            for (var i = 0; i < Pins.Count; i++)
            {
                if (string.Equals(Pins[i], pinId, StringComparison.Ordinal))
                {
                    return Torques[i];
                }
            }

            throw new KeyNotFoundException($"No holding torque was solved for pin '{pinId}'.");
        }
    }
}
