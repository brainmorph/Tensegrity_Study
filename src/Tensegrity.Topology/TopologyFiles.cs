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
    /// <summary>
    /// Loads a topology file, letting <paramref name="edit"/> change the
    /// document first (for example to add perturbations). Errors in the
    /// edited document are reported against <paramref name="path"/>.
    /// </summary>
    public static PlanarModel Load(string path, Action<TopologyDocument>? edit = null)
    {
        var document = RunFiles.ReadTopology(path);
        edit?.Invoke(document);
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
