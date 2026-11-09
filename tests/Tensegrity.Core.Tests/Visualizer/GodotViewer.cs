using System.Diagnostics;
using Tensegrity.Testing;

namespace Tensegrity.Core.Tests.Visualizer;

/// <summary>
/// Builds and launches the Godot viewer for headless checks. The Godot
/// executable comes from <c>TENSEGRITY_GODOT</c>, else the install path
/// in PLAN.md section 3. Tests that need it are skipped when it is absent.
/// </summary>
internal static class GodotViewer
{
    public const string ExecutableVariable = "TENSEGRITY_GODOT";

    private static readonly TimeSpan Timeout = TimeSpan.FromMinutes(3);
    private static readonly Lazy<string> BuildOutput = new(Build);

    public static string ProjectDirectory => Path.Combine(RepoPaths.Root, "godot");

    /// <summary>Where the executable is looked for.</summary>
    public static string ExecutablePath =>
        Environment.GetEnvironmentVariable(ExecutableVariable) is { Length: > 0 } path
            ? path
            : Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                "Desktop",
                "Godot_v4.7-stable_mono_linux_x86_64",
                "Godot_v4.7-stable_mono_linux.x86_64");

    public static bool IsAvailable => File.Exists(ExecutablePath);

    /// <summary>Builds the viewer once per test run, so the check never runs stale code.</summary>
    public static void EnsureBuilt() => _ = BuildOutput.Value;

    /// <summary>Runs the viewer headless with user arguments; returns its exit code and output.</summary>
    public static (int ExitCode, string Stdout, string Stderr) RunHeadless(params string[] userArgs)
    {
        EnsureBuilt();
        return Run(ExecutablePath, ["--headless", "--path", ProjectDirectory, "--", .. userArgs]);
    }

    private static string Build()
    {
        var dotnet = Environment.GetEnvironmentVariable("DOTNET_HOST_PATH") is { Length: > 0 } host ? host : "dotnet";
        var (exit, stdout, stderr) = Run(
            dotnet,
            ["build", Path.Combine(ProjectDirectory, "TensegrityViewer.csproj"), "-nologo", "-v", "q", "-nodeReuse:false"]);
        if (exit != 0)
        {
            throw new InvalidOperationException($"Building the viewer failed ({exit}):\n{stdout}\n{stderr}");
        }

        return stdout;
    }

    private static (int ExitCode, string Stdout, string Stderr) Run(string file, IEnumerable<string> args)
    {
        var start = new ProcessStartInfo(file)
        {
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            WorkingDirectory = RepoPaths.Root,
        };
        foreach (var arg in args)
        {
            start.ArgumentList.Add(arg);
        }

        // The test host's MSBuild settings must not leak into the nested build.
        foreach (var key in start.Environment.Keys.Where(k => k.StartsWith("MSBuild", StringComparison.OrdinalIgnoreCase)).ToList())
        {
            start.Environment.Remove(key);
        }

        using var process = Process.Start(start) ?? throw new InvalidOperationException($"Could not start {file}.");
        var stdout = process.StandardOutput.ReadToEndAsync();
        var stderr = process.StandardError.ReadToEndAsync();
        if (!process.WaitForExit(Timeout))
        {
            process.Kill(entireProcessTree: true);
            throw new TimeoutException($"{file} did not finish within {Timeout}.");
        }

        return (process.ExitCode, stdout.Result, stderr.Result);
    }
}

/// <summary>A theory that needs the Godot executable; skipped when it is absent.</summary>
public sealed class GodotTheoryAttribute : TheoryAttribute
{
    public GodotTheoryAttribute()
    {
        if (!GodotViewer.IsAvailable)
        {
            Skip = $"Godot not found at {GodotViewer.ExecutablePath}; set {GodotViewer.ExecutableVariable} to run the render-fidelity check.";
        }
    }
}
