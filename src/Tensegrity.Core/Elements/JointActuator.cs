namespace Tensegrity.Core.Elements;

/// <summary>
/// A joint actuator, the muscle stand-in of PLAN.md 4.1: a torque applied
/// across a pin joint, following a <see cref="Schedule"/>.
///
/// The torque τ acts on the pin's end <see cref="PinJoint.A"/> and −τ on end
/// <see cref="PinJoint.B"/>, so a positive τ turns A counterclockwise
/// relative to B. A ground end takes no torque. The pair is a pure couple:
/// it adds no net force, and on a free system it leaves the total angular
/// momentum unchanged. Its power is τ·(ω_A − ω_B).
/// </summary>
public sealed class JointActuator
{
    public JointActuator(string id, string pinId, Schedule schedule)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentException.ThrowIfNullOrWhiteSpace(pinId);
        ArgumentNullException.ThrowIfNull(schedule);
        Id = id;
        PinId = pinId;
        Schedule = schedule;
    }

    public string Id { get; }

    /// <summary>The pin the torque acts across.</summary>
    public string PinId { get; }

    public Schedule Schedule { get; }
}
