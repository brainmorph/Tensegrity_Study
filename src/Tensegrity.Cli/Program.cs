namespace Tensegrity.Cli;

internal static class Program
{
    private static int Main(string[] args) => new CliApp(Console.Out, Console.Error).Run(args);
}
