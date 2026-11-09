using Tensegrity.Core.Output;
using Tensegrity.RunFormat;

namespace Tensegrity.Topology;

/// <summary>
/// JSON topology files (PLAN.md 9.7): load a file into a
/// <see cref="PlanarModel"/>, and save a model, for example one built by a
/// generator, as a file. Loading a saved model gives back the same system
/// and initial state bit for bit.
/// </summary>
public static class TopologyFiles
{
    public static PlanarModel Load(string path)
    {
        var document = RunFiles.ReadTopology(path);
        try
        {
            return FromDocument(document);
        }
        catch (Exception e) when (e is FormatException or ArgumentException)
        {
            throw new InvalidDataException($"{path}: {e.Message}", e);
        }
    }

    public static PlanarModel FromDocument(TopologyDocument document)
    {
        var (system, state) = TopologyMapping.FromDocument(document);
        return new PlanarModel(document.Name, system, state, document.Description, document.Simulation);
    }

    public static TopologyDocument ToDocument(PlanarModel model) =>
        TopologyMapping.ToDocument(model.System, model.InitialState, model.Name, model.Description, model.Simulation);

    /// <summary>The exact text <see cref="Save"/> writes.</summary>
    public static string Serialize(PlanarModel model) => RunFiles.SerializeTopology(ToDocument(model));

    public static void Save(PlanarModel model, string path) => File.WriteAllText(path, Serialize(model));
}
