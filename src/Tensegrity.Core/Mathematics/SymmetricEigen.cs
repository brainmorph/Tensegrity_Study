namespace Tensegrity.Core.Mathematics;

/// <summary>
/// Eigen-decomposition of small dense symmetric matrices, A = V·diag(λ)·Vᵀ,
/// by cyclic Jacobi rotations, and the minimum-norm least-squares solve it
/// gives for singular systems. Matrices are row-major <c>n × n</c> spans.
/// Written here rather than taken from a library (PLAN.md 1a,
/// constraint 3); the systems it serves are a few rows at most, where
/// Jacobi is simple and accurate to round-off.
/// </summary>
public static class SymmetricEigen
{
    private const int MaxSweeps = 100;

    /// <summary>
    /// Eigenvalues of <paramref name="matrix"/> into <paramref name="values"/>
    /// and the matching unit eigenvectors as the columns of
    /// <paramref name="vectors"/> (row-major, so vector j is
    /// <c>vectors[i·n + j]</c> over i). Only the lower triangle of the
    /// matrix is read; the matrix itself is not changed.
    /// </summary>
    public static void Decompose(ReadOnlySpan<double> matrix, int n, Span<double> values, Span<double> vectors)
    {
        CheckShape(matrix.Length, n);
        CheckShape(vectors.Length, n);
        if (values.Length != n)
        {
            throw new ArgumentException($"Eigenvalue length {values.Length} does not match n = {n}.", nameof(values));
        }

        var a = new double[n * n];
        for (var i = 0; i < n; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                a[(i * n) + j] = matrix[(i * n) + j];
                a[(j * n) + i] = matrix[(i * n) + j];
            }
        }

        vectors.Clear();
        for (var i = 0; i < n; i++)
        {
            vectors[(i * n) + i] = 1.0;
        }

        for (var sweep = 0; sweep < MaxSweeps; sweep++)
        {
            var offDiagonal = 0.0;
            var diagonal = 0.0;
            for (var i = 0; i < n; i++)
            {
                diagonal += a[(i * n) + i] * a[(i * n) + i];
                for (var j = 0; j < i; j++)
                {
                    offDiagonal += a[(i * n) + j] * a[(i * n) + j];
                }
            }

            if (!(offDiagonal > 1e-32 * diagonal))
            {
                break;
            }

            for (var p = 0; p < n - 1; p++)
            {
                for (var q = p + 1; q < n; q++)
                {
                    Rotate(a, vectors, n, p, q);
                }
            }
        }

        for (var i = 0; i < n; i++)
        {
            values[i] = a[(i * n) + i];
        }
    }

    /// <summary>
    /// Solves A·x = b in the least-squares sense with the smallest |x|, in
    /// place (<paramref name="rhs"/> becomes x), for a symmetric positive
    /// semi-definite A: x = Σ (vᵀ·b / λ)·v over the eigenpairs with
    /// λ above <paramref name="relativeTolerance"/> times the largest. Directions
    /// with smaller λ are treated as the null space and get no component.
    /// Returns the rank, the number of eigenpairs kept.
    /// </summary>
    public static int SolveMinimumNorm(ReadOnlySpan<double> matrix, int n, Span<double> rhs, double relativeTolerance = 1e-12)
    {
        CheckShape(matrix.Length, n);
        if (rhs.Length != n)
        {
            throw new ArgumentException($"Right-hand side length {rhs.Length} does not match n = {n}.", nameof(rhs));
        }

        var values = new double[n];
        var vectors = new double[n * n];
        Decompose(matrix, n, values, vectors);

        var largest = 0.0;
        foreach (var value in values)
        {
            largest = Math.Max(largest, value);
        }

        var floor = relativeTolerance * largest;
        var solution = new double[n];
        var rank = 0;
        for (var j = 0; j < n; j++)
        {
            if (!(values[j] > floor))
            {
                continue;
            }

            rank++;
            var projection = 0.0;
            for (var i = 0; i < n; i++)
            {
                projection += vectors[(i * n) + j] * rhs[i];
            }

            var scale = projection / values[j];
            for (var i = 0; i < n; i++)
            {
                solution[i] += scale * vectors[(i * n) + j];
            }
        }

        solution.CopyTo(rhs);
        return rank;
    }

    /// <summary>One Jacobi rotation in the (p, q) plane that zeroes a[p, q].</summary>
    private static void Rotate(double[] a, Span<double> vectors, int n, int p, int q)
    {
        var apq = a[(p * n) + q];
        if (apq == 0)
        {
            return;
        }

        var app = a[(p * n) + p];
        var aqq = a[(q * n) + q];
        var theta = (aqq - app) / (2 * apq);
        var t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt((theta * theta) + 1));
        if (theta == 0)
        {
            t = 1;
        }

        var c = 1 / Math.Sqrt((t * t) + 1);
        var s = t * c;

        for (var k = 0; k < n; k++)
        {
            var akp = a[(k * n) + p];
            var akq = a[(k * n) + q];
            a[(k * n) + p] = (c * akp) - (s * akq);
            a[(k * n) + q] = (s * akp) + (c * akq);
        }

        for (var k = 0; k < n; k++)
        {
            var apk = a[(p * n) + k];
            var aqk = a[(q * n) + k];
            a[(p * n) + k] = (c * apk) - (s * aqk);
            a[(q * n) + k] = (s * apk) + (c * aqk);
        }

        for (var k = 0; k < n; k++)
        {
            var vkp = vectors[(k * n) + p];
            var vkq = vectors[(k * n) + q];
            vectors[(k * n) + p] = (c * vkp) - (s * vkq);
            vectors[(k * n) + q] = (s * vkp) + (c * vkq);
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
