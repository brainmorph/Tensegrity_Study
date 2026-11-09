using System.Reflection;
using System.Runtime.Versioning;
using System.Xml.Linq;
using Tensegrity.Testing;

namespace Tensegrity.Core.Tests;

/// <summary>
/// Guards the project boundaries in PLAN.md sections 1a (constraint 5) and 5:
/// RunFormat is a leaf that Godot can reference without reaching the solver,
/// and nothing on the simulation side depends on Godot.
/// </summary>
public class ArchitectureTests
{
    private static readonly string[] SimulationProjects =
    [
        "Tensegrity.Core",
        "Tensegrity.RunFormat",
        "Tensegrity.Topology",
        "Tensegrity.Cli",
    ];

    [Fact]
    public void RunFormatProjectHasNoProjectOrPackageReferences()
    {
        var project = LoadProject("Tensegrity.RunFormat");

        Assert.Empty(project.Descendants("ProjectReference"));
        Assert.Empty(project.Descendants("PackageReference"));
    }

    [Fact]
    public void RunFormatAssemblyReferencesNoOtherTensegrityAssembly()
    {
        var references = Assembly.Load("Tensegrity.RunFormat").GetReferencedAssemblies();

        Assert.DoesNotContain(references, r => r.Name!.StartsWith("Tensegrity", StringComparison.Ordinal));
    }

    [Fact]
    public void RunFormatTargetsNet8SoGodotCanReferenceIt()
    {
        var framework = Assembly.Load("Tensegrity.RunFormat")
            .GetCustomAttribute<TargetFrameworkAttribute>()?.FrameworkName;

        Assert.Equal(".NETCoreApp,Version=v8.0", framework);
    }

    [Theory]
    [MemberData(nameof(SimulationProjectNames))]
    public void SimulationProjectDoesNotReferenceGodot(string name)
    {
        var references = LoadProject(name)
            .Descendants()
            .Where(e => e.Name.LocalName is "ProjectReference" or "PackageReference" or "Reference")
            .Select(e => (string?)e.Attribute("Include") ?? string.Empty);

        Assert.DoesNotContain(references, r => r.Contains("Godot", StringComparison.OrdinalIgnoreCase));
    }

    [Theory]
    [InlineData("Tensegrity.Core")]
    [InlineData("Tensegrity.RunFormat")]
    [InlineData("Tensegrity.Topology")]
    public void SimulationAssemblyDoesNotReferenceGodot(string name)
    {
        var references = Assembly.Load(name).GetReferencedAssemblies();

        Assert.DoesNotContain(references, r => r.Name!.Contains("Godot", StringComparison.OrdinalIgnoreCase));
    }

    public static TheoryData<string> SimulationProjectNames() => new(SimulationProjects);

    private static XDocument LoadProject(string name) =>
        XDocument.Load(Path.Combine(RepoPaths.Root, "src", name, name + ".csproj"));
}
