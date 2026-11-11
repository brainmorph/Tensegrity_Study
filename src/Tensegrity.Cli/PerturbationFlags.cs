using System.Globalization;
using Tensegrity.RunFormat;

namespace Tensegrity.Cli;

/// <summary>
/// Perturbations given on the command line (PLAN.md M6), added to a
/// topology document before it is built, so a run can be kicked without
/// editing its file. They become ordinary loads and ground schedules, so
/// the run manifest records them like any others.
/// <list type="bullet">
/// <item><c>--impulse LINK,X,Y,JX,JY,START,DURATION</c>: a triangular force
/// pulse delivering (JX, JY) N·s, world frame, at body point (X, Y) of
/// LINK.</item>
/// <item><c>--force LINK,X,Y,T,FX,FY[,T,FX,FY...]</c>: a world-frame force at
/// body point (X, Y), piecewise-linear through the (T, FX, FY) breakpoints
/// and held before the first and after the last, as in a file's
/// <c>loads</c>.</item>
/// <item><c>--ground-drop START,DURATION,DEPTH</c>: the ground line falls
/// DEPTH metres at a steady rate over DURATION seconds from START. The
/// topology must have a fixed ground.</item>
/// </list>
/// <c>--impulse</c> and <c>--force</c> may be repeated.
/// </summary>
internal static class PerturbationFlags
{
    public static readonly string[] Names = ["impulse", "force", "ground-drop"];

    public static readonly string[] Repeatable = ["impulse", "force"];

    /// <summary>Adds the perturbation flags in <paramref name="options"/> to <paramref name="document"/>.</summary>
    /// <exception cref="CliApp.UsageException">A flag is malformed or names something the topology lacks.</exception>
    public static void Apply(TopologyDocument document, CliApp.Options options)
    {
        var loads = document.Loads ?? [];
        var count = 0;
        foreach (var flag in options.All("impulse"))
        {
            var (link, x, y, rest) = LinkPoint(document, "impulse", flag);
            if (rest.Length != 4)
            {
                throw new CliApp.UsageException($"--impulse '{flag}' needs LINK,X,Y,JX,JY,START,DURATION.");
            }

            double jx = rest[0], jy = rest[1], start = rest[2], duration = rest[3];
            if (!(duration > 0))
            {
                throw new CliApp.UsageException($"--impulse '{flag}' needs a positive duration.");
            }

            // Matches ExternalLoad.Pulse: zero, a peak of 2·J/τ at mid-pulse, and zero.
            var peak = 2.0 / duration;
            loads.Add(Load(document, "impulse", ++count, link, x, y,
            [
                new LoadPointDocument { Time = start },
                new LoadPointDocument { Time = start + (duration / 2), Fx = peak * jx, Fy = peak * jy },
                new LoadPointDocument { Time = start + duration },
            ]));
        }

        count = 0;
        foreach (var flag in options.All("force"))
        {
            var (link, x, y, rest) = LinkPoint(document, "force", flag);
            if (rest.Length == 0 || rest.Length % 3 != 0)
            {
                throw new CliApp.UsageException($"--force '{flag}' needs LINK,X,Y followed by one or more T,FX,FY breakpoints.");
            }

            var schedule = new List<LoadPointDocument>();
            for (var i = 0; i < rest.Length; i += 3)
            {
                if (i > 0 && !(rest[i] > rest[i - 3]))
                {
                    throw new CliApp.UsageException($"--force '{flag}' needs strictly increasing times.");
                }

                schedule.Add(new LoadPointDocument { Time = rest[i], Fx = rest[i + 1], Fy = rest[i + 2] });
            }

            loads.Add(Load(document, "force", ++count, link, x, y, schedule));
        }

        document.Loads = loads.Count == 0 ? document.Loads : loads;

        if (options.Value("ground-drop") is { } drop)
        {
            var values = Numbers("ground-drop", drop, drop.Split(','));
            if (values.Length != 3 || !(values[1] > 0) || !(values[2] > 0))
            {
                throw new CliApp.UsageException($"--ground-drop '{drop}' needs START,DURATION,DEPTH with a positive duration and depth.");
            }

            var ground = document.Ground ?? throw new CliApp.UsageException($"--ground-drop: topology '{document.Name}' has no ground line.");
            if (ground.HeightSchedule is not null)
            {
                throw new CliApp.UsageException($"--ground-drop: topology '{document.Name}' already moves its ground line.");
            }

            ground.HeightSchedule =
            [
                new HeightPointDocument { Time = values[0], Height = ground.Height },
                new HeightPointDocument { Time = values[0] + values[1], Height = ground.Height - values[2] },
            ];
        }
    }

    private static (string Link, double X, double Y, double[] Values) LinkPoint(TopologyDocument document, string flag, string value)
    {
        var parts = value.Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length < 3)
        {
            throw new CliApp.UsageException($"--{flag} '{value}' needs LINK,X,Y first.");
        }

        var link = parts[0];
        if (!document.Links.Exists(l => string.Equals(l.Id, link, StringComparison.Ordinal)))
        {
            throw new CliApp.UsageException($"--{flag}: topology '{document.Name}' has no link '{link}'.");
        }

        var numbers = Numbers(flag, value, parts[1..]);
        return (link, numbers[0], numbers[1], numbers[2..]);
    }

    private static double[] Numbers(string flag, string value, string[] parts)
    {
        var numbers = new double[parts.Length];
        for (var i = 0; i < parts.Length; i++)
        {
            if (!double.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out numbers[i]) || !double.IsFinite(numbers[i]))
            {
                throw new CliApp.UsageException($"--{flag} '{value}': '{parts[i]}' is not a number.");
            }
        }

        return numbers;
    }

    private static LoadDocument Load(TopologyDocument document, string kind, int index, string link, double x, double y, List<LoadPointDocument> schedule)
    {
        var id = $"cli_{kind}_{index}";
        if (document.Loads?.Exists(l => string.Equals(l.Id, id, StringComparison.Ordinal)) == true)
        {
            throw new CliApp.UsageException($"--{kind}: topology '{document.Name}' already has a load '{id}'.");
        }

        return new LoadDocument { Id = id, Link = link, Point = [x, y], Schedule = schedule };
    }
}
