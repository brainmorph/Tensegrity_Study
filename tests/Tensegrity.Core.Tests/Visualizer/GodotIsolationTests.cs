using System.Xml.Linq;
using Tensegrity.Testing;

namespace Tensegrity.Core.Tests.Visualizer;

/// <summary>
/// PLAN.md section 7, test 15: the Godot project renders recorded runs
/// and nothing else. It references no solver project, uses no physics
/// node or physics server, and writes no files (constraint 5, section 1a).
/// </summary>
public class GodotIsolationTests
{
    private static string GodotRoot => Path.Combine(RepoPaths.Root, "godot");

    [Fact]
    public void GodotProjectBreaksNoIsolationRule()
    {
        var violations = GodotIsolation.Scan(GodotRoot).Select(v => v.ToString()).ToList();

        Assert.True(violations.Count == 0, "Isolation violations in godot/:\n" + string.Join('\n', violations));
    }

    [Fact]
    public void ScanCoversTheViewerSourcesAndProjectFiles()
    {
        // Guards against a scan that passes because it found nothing.
        var files = GodotIsolation.ScannedFiles(GodotRoot);

        Assert.Contains("project.godot", files);
        Assert.Contains("TensegrityViewer.csproj", files);
        Assert.Contains("Main.tscn", files);
        Assert.Contains(Path.Combine("Scripts", "Main.cs"), files);
        Assert.Contains(Path.Combine("Scripts", "RunScene.cs"), files);
        Assert.DoesNotContain(files, f => f.StartsWith(".godot", StringComparison.Ordinal));
    }

    [Fact]
    public void ViewerReferencesOnlyTheRunFormatProject()
    {
        foreach (var path in Directory.EnumerateFiles(GodotRoot, "*.csproj", SearchOption.TopDirectoryOnly))
        {
            var project = XDocument.Load(path);
            var references = project.Descendants("ProjectReference")
                .Select(e => Path.GetFileName(((string?)e.Attribute("Include") ?? string.Empty).Replace('\\', '/')))
                .ToList();

            Assert.Equal(["Tensegrity.RunFormat.csproj"], references);
            Assert.Empty(project.Descendants("PackageReference"));
            Assert.Empty(project.Descendants("Reference"));
        }
    }

    [Fact]
    public void BothGodotPhysicsEnginesAreTheDummyServer()
    {
        var settings = File.ReadAllLines(Path.Combine(GodotRoot, "project.godot")).Select(l => l.Trim()).ToList();

        Assert.Contains("2d/physics_engine=\"Dummy\"", settings);
        Assert.Contains("3d/physics_engine=\"Dummy\"", settings);
    }

    [Theory]
    [InlineData("Scripts/A.cs", "using Tensegrity.Core.Systems;", "solver-reference")]
    [InlineData("A.csproj", "<ProjectReference Include=\"../src/Tensegrity.Topology/Tensegrity.Topology.csproj\" />", "solver-reference")]
    [InlineData("Main.tscn", "[node name=\"Body\" type=\"RigidBody2D\" parent=\".\"]", "physics-node")]
    [InlineData("Scripts/A.cs", "var shape = new CollisionShape2D();", "physics-node")]
    [InlineData("Scripts/A.cs", "AddChild(new PinJoint2D());", "physics-node")]
    [InlineData("Scripts/A.cs", "public partial class Sensor : Area2D", "physics-node")]
    [InlineData("Scripts/A.cs", "PhysicsServer2D.BodySetState(rid, state, value);", "physics-server")]
    [InlineData("Scripts/A.cs", "var space = GetWorld2D().DirectSpaceState as PhysicsDirectSpaceState2D;", "physics-server")]
    [InlineData("Scripts/A.cs", "public override void _PhysicsProcess(double delta)", "physics-callback")]
    [InlineData("a.gd", "func _physics_process(delta):", "physics-callback")]
    [InlineData("Scripts/A.cs", "File.WriteAllText(path, text);", "file-write")]
    [InlineData("Scripts/A.cs", "using var stream = new FileStream(path, FileMode.Create);", "file-write")]
    [InlineData("Scripts/A.cs", "using var f = FileAccess.Open(path, FileAccess.ModeFlags.Write);", "file-write")]
    [InlineData("Scripts/A.cs", "DirAccess.MakeDirAbsolute(path);", "file-write")]
    [InlineData("Scripts/A.cs", "Directory.CreateDirectory(path);", "file-write")]
    [InlineData("a.gd", "var f = FileAccess.open(path, FileAccess.WRITE)", "file-write")]
    [InlineData("Scripts/A.cs", "OS.Execute(\"dotnet\", args);", "process-launch")]
    [InlineData("Scripts/A.cs", "Process.Start(\"dotnet\", \"run\");", "process-launch")]
    public void RuleFlagsBannedCode(string file, string line, string rule)
    {
        var violations = GodotIsolation.ScanText(file, line).ToList();

        Assert.Contains(violations, v => v.Rule == rule);
    }

    [Theory]
    [InlineData("using Tensegrity.RunFormat;")]
    [InlineData("<ProjectReference Include=\"../src/Tensegrity.RunFormat/Tensegrity.RunFormat.csproj\" />")]
    [InlineData("var run = RunReader.Open(directory);")]
    [InlineData("var bytes = File.ReadAllBytes(path);")]
    [InlineData("if (Directory.Exists(runs))")]
    [InlineData("public override void _Process(double delta)")]
    [InlineData("public Func<Rect2>? FreeArea { get; set; }")]
    [InlineData("2d/physics_engine=\"Dummy\"")]
    [InlineData("slot.View.Rotation = values[slot.Theta];")]
    public void RuleAllowsRenderingCode(string line)
    {
        Assert.Empty(GodotIsolation.ScanText("Scripts/A.cs", line));
    }
}
