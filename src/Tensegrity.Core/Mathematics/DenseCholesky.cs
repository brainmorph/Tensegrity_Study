namespace Tensegrity.Core.Mathematics;

/// <summary>
/// In-place Cholesky factorization and solve for small dense symmetric
/// positive-definite systems, A = L·Lᵀ. Matrices are row-major
/// <c>n × n</c> spans. Written here rather than taken from a library
/// (PLAN.md 1a, constraint 3); the constraint systems it serves are a few
/// rows at most.
/// </summary>
public static class DenseCholesky
{
    /// <summary>
    /// Overwrites the lower triangle of <paramref name="matrix"/> with L.
    /// The strict upper triangle is not read or written. Returns false if a
    /// pivot is not positive or falls below <paramref name="relativeTolerance"/>
    /// times the largest diagonal entry, which means the matrix is singular
    /// or not positive definite (for constraints: redundant or degenerate).
    /// </summary>
    public static bool TryFactor(Span<double> matrix, int n, double relativeTolerance = 1e-12)
    {
        CheckShape(matrix.Length, n);

        var maxDiagonal = 0.0;
        for (var i = 0; i < n; i++)
        {
            maxDiagonal = Math.Max(maxDiagonal, Math.Abs(matrix[(i * n) + i]));
        }

        var floor = relativeTolerance * maxDiagonal;
        for (var j = 0; j < n; j++)
        {
            var pivot = matrix[(j * n) + j];
            for (var k = 0; k < j; k++)
            {
                pivot -= matrix[(j * n) + k] * matrix[(j * n) + k];
            }

            if (!(pivot > floor))
            {
                return false;
            }

            var ljj = Math.Sqrt(pivot);
            matrix[(j * n) + j] = ljj;
            for (var i = j + 1; i < n; i++)
            {
                var sum = matrix[(i * n) + j];
                for (var k = 0; k < j; k++)
                {
                    sum -= matrix[(i * n) + k] * matrix[(j * n) + k];
                }

                matrix[(i * n) + j] = sum / ljj;
            }
        }

        return true;
    }

    /// <summary>
    /// Solves L·Lᵀ·x = b in place (<paramref name="rhs"/> becomes x), using a
    /// factor produced by <see cref="TryFactor"/>.
    /// </summary>
    public static void Solve(ReadOnlySpan<double> factor, int n, Span<double> rhs)
    {
        CheckShape(factor.Length, n);
        if (rhs.Length != n)
        {
            throw new ArgumentException($"Right-hand side length {rhs.Length} does not match n = {n}.", nameof(rhs));
        }

        // Forward: L·y = b.
        for (var i = 0; i < n; i++)
        {
            var sum = rhs[i];
            for (var k = 0; k < i; k++)
            {
                sum -= factor[(i * n) + k] * rhs[k];
            }

            rhs[i] = sum / factor[(i * n) + i];
        }

        // Backward: Lᵀ·x = y.
        for (var i = n - 1; i >= 0; i--)
        {
            var sum = rhs[i];
            for (var k = i + 1; k < n; k++)
            {
                sum -= factor[(k * n) + i] * rhs[k];
            }

            rhs[i] = sum / factor[(i * n) + i];
        }
    }

    private static void CheckShape(int length, int n)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(n);
        if (length != n * n)
        {
            throw new ArgumentException($"Matrix length {length} is not {n}×{n}.");
        }
    }
}
