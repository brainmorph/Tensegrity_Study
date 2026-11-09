namespace Tensegrity.Testing;

/// <summary>Locates files in the source tree from a running test.</summary>
internal static class RepoPaths
{
    private const string SolutionFile = "Tensegrity.sln";

    private static readonly Lazy<string> RootLazy = new(FindRoot);

    /// <summary>Absolute path of the repository root (the folder holding the solution).</summary>
    public static string Root => RootLazy.Value;

    /// <summary>Absolute path of the golden baseline folder, tests/golden.</summary>
    public static string GoldenRoot => Path.Combine(Root, "tests", "golden");

    private static string FindRoot()
    {
        for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            if (File.Exists(Path.Combine(dir.FullName, SolutionFile)))
            {
                return dir.FullName;
            }
        }

        throw new InvalidOperationException(
            $"Could not find {SolutionFile} above {AppContext.BaseDirectory}.");
    }
}
