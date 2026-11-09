namespace Tensegrity.Viewer;

/// <summary>
/// Command-line options, passed after Godot's own arguments and a
/// <c>--</c> separator:
/// <code>godot --path godot -- --run runs/my-run [--fidelity-dump]</code>
/// </summary>
public sealed class ViewerOptions
{
    public const string Usage = "usage: -- [--run <run directory>] [--fidelity-dump]";

    /// <summary>The run directory to open, absolute; null opens the folder picker.</summary>
    public string? RunDirectory { get; private init; }

    /// <summary>
    /// Headless render-fidelity check (PLAN.md section 7, test 16): show
    /// every frame, print the scene's transforms, and quit.
    /// </summary>
    public bool FidelityDump { get; private init; }

    /// <summary>
    /// Parses the user arguments. A relative run path is resolved against
    /// the directory Godot was launched from (<c>PWD</c>), because Godot
    /// changes its working directory to the project folder.
    /// </summary>
    public static ViewerOptions Parse(IReadOnlyList<string> args, string launchDirectory)
    {
        string? run = null;
        var dump = false;
        for (var i = 0; i < args.Count; i++)
        {
            switch (args[i])
            {
                case "--run" when i + 1 < args.Count:
                    run = args[++i];
                    break;
                case "--fidelity-dump":
                    dump = true;
                    break;
                default:
                    throw new ArgumentException($"Unknown or incomplete argument '{args[i]}'. {Usage}");
            }
        }

        if (dump && run is null)
        {
            throw new ArgumentException($"--fidelity-dump needs --run. {Usage}");
        }

        return new ViewerOptions
        {
            RunDirectory = run is null ? null : Path.GetFullPath(run, launchDirectory),
            FidelityDump = dump,
        };
    }

    /// <summary>The shell's working directory when Godot was launched.</summary>
    public static string LaunchDirectory() =>
        System.Environment.GetEnvironmentVariable("PWD") is { Length: > 0 } pwd && Path.IsPathRooted(pwd)
            ? pwd
            : Directory.GetCurrentDirectory();
}
