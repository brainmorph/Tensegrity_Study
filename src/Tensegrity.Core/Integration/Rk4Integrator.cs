namespace Tensegrity.Core.Integration;

/// <summary>
/// Classical fixed-step fourth-order Runge–Kutta:
/// <code>
/// k1 = f(t,        y)
/// k2 = f(t + h/2,  y + h/2·k1)
/// k3 = f(t + h/2,  y + h/2·k2)
/// k4 = f(t + h,    y + h·k3)
/// y ← y + h/6·(k1 + 2·k2 + 2·k3 + k4)
/// </code>
/// Arithmetic runs in a fixed order with no parallelism, so results are
/// bit-reproducible on a given platform. Scratch buffers are reused between
/// steps, so one instance must not be shared across threads.
/// </summary>
public sealed class Rk4Integrator : IIntegrator
{
    private double[] k1 = [];
    private double[] k2 = [];
    private double[] k3 = [];
    private double[] k4 = [];
    private double[] stage = [];

    public string Name => "rk4";

    public void Step(SystemState state, double dt, Derivatives derivatives)
    {
        ArgumentNullException.ThrowIfNull(state);
        ArgumentNullException.ThrowIfNull(derivatives);
        if (!double.IsFinite(dt) || dt <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "Must be finite and positive.");
        }

        EnsureCapacity(state.Length);
        var y = state.Values;
        var n = y.Length;
        var t = state.Time;
        var half = 0.5 * dt;

        derivatives(t, y, k1);

        for (var i = 0; i < n; i++)
        {
            stage[i] = y[i] + (half * k1[i]);
        }

        derivatives(t + half, stage, k2);

        for (var i = 0; i < n; i++)
        {
            stage[i] = y[i] + (half * k2[i]);
        }

        derivatives(t + half, stage, k3);

        for (var i = 0; i < n; i++)
        {
            stage[i] = y[i] + (dt * k3[i]);
        }

        derivatives(t + dt, stage, k4);

        var sixth = dt / 6.0;
        for (var i = 0; i < n; i++)
        {
            y[i] += sixth * (k1[i] + (2.0 * k2[i]) + (2.0 * k3[i]) + k4[i]);
        }

        state.Time = t + dt;
    }

    private void EnsureCapacity(int n)
    {
        if (k1.Length == n)
        {
            return;
        }

        k1 = new double[n];
        k2 = new double[n];
        k3 = new double[n];
        k4 = new double[n];
        stage = new double[n];
    }
}
