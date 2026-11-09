using System.Text.RegularExpressions;

namespace Tensegrity.Core.Tests.Visualizer;

/// <summary>
/// Text rules for the Godot project (PLAN.md section 1a, constraint 5):
/// the viewer reaches no solver project, uses no physics, and writes no
/// files. The rules match code, scenes, resources and build files; they
/// do not read prose (Markdown), so documentation may name what is banned.
/// </summary>
internal static partial class GodotIsolation
{
    /// <summary>File types scanned: code, scenes, resources, project and build files.</summary>
    public static readonly string[] ScannedExtensions =
        [".cs", ".gd", ".tscn", ".tres", ".godot", ".cfg", ".csproj", ".props", ".targets", ".sln", ".slnx"];

    /// <summary>Generated or cached folders Godot and MSBuild own.</summary>
    private static readonly string[] SkippedFolders = [".godot", ".import", "bin", "obj"];

    public static readonly IReadOnlyList<Rule> Rules =
    [
        new("solver-reference", SolverReference()),
        new("physics-node", PhysicsNode()),
        new("physics-server", PhysicsServer()),
        new("physics-callback", PhysicsCallback()),
        new("file-write", FileWrite()),
        new("process-launch", ProcessLaunch()),
    ];

    /// <summary>Every scanned file under <paramref name="root"/>, as paths relative to it.</summary>
    public static IReadOnlyList<string> ScannedFiles(string root) =>
        Directory.EnumerateFiles(root, "*", SearchOption.AllDirectories)
            .Select(path => Path.GetRelativePath(root, path))
            .Where(path => !path.Split(Path.DirectorySeparatorChar).SkipLast(1).Any(SkippedFolders.Contains))
            .Where(path => ScannedExtensions.Contains(Path.GetExtension(path), StringComparer.OrdinalIgnoreCase))
            .Order(StringComparer.Ordinal)
            .ToList();

    public static IEnumerable<Violation> Scan(string root) =>
        ScannedFiles(root).SelectMany(file => ScanText(file, File.ReadAllText(Path.Combine(root, file))));

    public static IEnumerable<Violation> ScanText(string file, string text)
    {
        var lines = text.Split('\n');
        for (var i = 0; i < lines.Length; i++)
        {
            foreach (var rule in Rules)
            {
                var match = rule.Pattern.Match(lines[i]);
                if (match.Success)
                {
                    yield return new Violation(file, i + 1, rule.Name, match.Value);
                }
            }
        }
    }

    // Any solver-side project: the viewer may use Tensegrity.RunFormat only.
    [GeneratedRegex(@"\bTensegrity\.(?:Core|Topology|Cli)\b")]
    private static partial Regex SolverReference();

    // Bodies, areas, shapes, joints and queries of Godot's physics, in C#, GDScript and scene files.
    [GeneratedRegex(@"\b(?:RigidBody|CharacterBody|StaticBody|AnimatableBody|PhysicsBody|SoftBody|VehicleBody|VehicleWheel|PhysicalBone|Area|CollisionShape|CollisionPolygon|CollisionObject|RayCast|ShapeCast|SpringArm|\w*Joint)[23]D\b")]
    private static partial Regex PhysicsNode();

    [GeneratedRegex(@"\bPhysics(?:Server[23]D\w*|Direct\w+|Material|RayQueryParameters\w*|ShapeQueryParameters\w*|PointQueryParameters\w*|TestMotion\w*)\b")]
    private static partial Regex PhysicsServer();

    [GeneratedRegex(@"\b(?:_PhysicsProcess|_IntegrateForces|_physics_process|_integrate_forces)\b")]
    private static partial Regex PhysicsCallback();

    // Anything that can create, change or delete a file or folder.
    [GeneratedRegex(@"\bFile\.(?:Write\w*|Append\w*|Create\w*|Delete|Move|Copy|Replace|Open|OpenWrite|OpenHandle|Set\w+)\b|\bDirectory\.(?:Create\w*|Delete|Move|Set\w+)\b|\b(?:FileStream|StreamWriter|BinaryWriter|FileInfo|DirectoryInfo|DirAccess|ResourceSaver|ConfigFile)\b|\bModeFlags\.(?:Write|ReadWrite|WriteRead)\b|\bFileAccess\.(?:WRITE|READ_WRITE|WRITE_READ)\b|\bProjectSettings\.Save\w*\b")]
    private static partial Regex FileWrite();

    // Launching processes could run the simulator or write files indirectly.
    [GeneratedRegex(@"\bOS\.(?:Execute\w*|CreateProcess|CreateInstance|MoveToTrash|execute\w*|create_process|create_instance|move_to_trash)\b|\bProcess\.Start\b|\bProcessStartInfo\b")]
    private static partial Regex ProcessLaunch();

    public sealed record Rule(string Name, Regex Pattern);

    public sealed record Violation(string File, int Line, string Rule, string Text)
    {
        public override string ToString() => $"{File}:{Line} [{Rule}] {Text}";
    }
}
