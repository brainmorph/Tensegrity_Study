using System.Text.Json;
using Tensegrity.Core.Output;
using Tensegrity.RunFormat;

namespace Tensegrity.Core.Analysis;

/// <summary>One cable setting in an <see cref="EffortSweep"/>.</summary>
/// <param name="Scale">Rest length as a multiple of the topology's own.</param>
/// <param name="RestLength">The cable's rest length at this scale, m.</param>
/// <param name="Tension">The cable's tension in the topology's initial pose at rest, N.</param>
/// <param name="Torques">Holding torque at each swept pin, N·m (+ on the pin's end A).</param>
/// <param name="MaxLinearAcceleration">Acceleration the pins cannot cancel, m/s²; zero when they hold the pose.</param>
/// <param name="MaxAngularAcceleration">Angular acceleration the pins cannot cancel, rad/s².</param>
public sealed record EffortPoint(
    double Scale,
    double RestLength,
    double Tension,
    IReadOnlyList<double> Torques,
    double MaxLinearAcceleration,
    double MaxAngularAcceleration);

/// <summary>
/// Effort curves (PLAN.md M5): the holding torque needed to keep a
/// topology's initial pose still (test 11's measure) as one cable's rest
/// length is scaled. The pose is taken at rest: any initial velocities in
/// the topology are dropped, so no damping or velocity terms enter. Zero torque means the cables hold the pose by
/// themselves; the slope shows how much effort a mis-set tendon costs. The
/// pose itself never changes, so a lengthened cable that goes slack
/// shows the full load falling on the joints.
/// </summary>
public static class EffortSweep
{
    public static IReadOnlyList<EffortPoint> Run(
        TopologyDocument document,
        string cableId,
        IReadOnlyList<string> pinIds,
        IEnumerable<double> scales)
    {
        ArgumentNullException.ThrowIfNull(document);
        ArgumentNullException.ThrowIfNull(pinIds);
        ArgumentNullException.ThrowIfNull(scales);
        var index = document.Cables.FindIndex(c => string.Equals(c.Id, cableId, StringComparison.Ordinal));
        if (index < 0)
        {
            throw new ArgumentException($"Topology '{document.Name}' has no cable '{cableId}'.", nameof(cableId));
        }

        var baseRest = document.Cables[index].RestLength;
        var points = new List<EffortPoint>();
        foreach (var scale in scales)
        {
            // Each point gets its own copy, so the caller's document is untouched.
            var copy = JsonSerializer.Deserialize<TopologyDocument>(RunFiles.SerializeTopology(document), RunFiles.Json)!;
            copy.Cables[index].RestLength = scale * baseRest;
            foreach (var link in copy.Links)
            {
                link.Twist = new TwistDocument();
            }

            var (system, state) = TopologyMapping.FromDocument(copy);
            var tensions = new double[system.Cables.Count];
            system.GetCableTensions(state, tensions);
            var hold = system.SolveHoldingTorques(0, state, pinIds);
            points.Add(new EffortPoint(scale, scale * baseRest, tensions[index], hold.Torques, hold.MaxLinearAcceleration, hold.MaxAngularAcceleration));
        }

        return points;
    }
}
