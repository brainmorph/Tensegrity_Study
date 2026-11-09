using System.Text.Encodings.Web;
using System.Text.Json;
using Tensegrity.Testing;

namespace Tensegrity.Regression.Harness;

/// <summary>
/// Writes baseline.json from a scenario's current output. Used only through
/// the explicit opt-in in <see cref="RegressionCheck"/>; never called
/// automatically.
/// </summary>
public static class GoldenWriter
{
    private static readonly JsonWriterOptions WriterOptions = new()
    {
        Indented = true,
        Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
    };

    public static void Write(string directory, IReferenceScenario scenario, IReadOnlyDictionary<string, double[]> output)
    {
        Directory.CreateDirectory(directory);
        using var stream = File.Create(Path.Combine(directory, GoldenBaseline.BaselineFile));
        using (var json = new Utf8JsonWriter(stream, WriterOptions))
        {
            json.WriteStartObject();
            json.WriteString("name", scenario.Name);
            json.WriteString("description", scenario.Description);
            json.WriteStartObject("quantities");
            foreach (var (name, values) in output.OrderBy(kv => kv.Key, StringComparer.Ordinal))
            {
                json.WriteStartArray(name);
                foreach (var value in values)
                {
                    // System.Text.Json writes the shortest round-trippable form.
                    json.WriteNumberValue(value);
                }

                json.WriteEndArray();
            }

            json.WriteEndObject();
            json.WriteEndObject();
        }

        stream.WriteByte((byte)'\n');
    }
}
