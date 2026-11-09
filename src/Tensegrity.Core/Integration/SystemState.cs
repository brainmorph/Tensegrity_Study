namespace Tensegrity.Core.Integration;

/// <summary>
/// Generic state for any first-order ODE system: a time and a flat vector of
/// doubles. The integrator knows nothing about what the entries mean, so the
/// same integrator serves 2D now and 3D later (PLAN.md section 5).
/// </summary>
public sealed class SystemState
{
    public SystemState(int length, double time = 0)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(length);
        Values = new double[length];
        Time = time;
    }

    public SystemState(double[] values, double time = 0)
    {
        ArgumentNullException.ThrowIfNull(values);
        Values = values;
        Time = time;
    }

    /// <summary>Simulation time in seconds.</summary>
    public double Time { get; set; }

    /// <summary>The state vector. Its length is fixed for the life of the state.</summary>
    public double[] Values { get; }

    public int Length => Values.Length;

    public SystemState Clone() => new((double[])Values.Clone(), Time);
}
