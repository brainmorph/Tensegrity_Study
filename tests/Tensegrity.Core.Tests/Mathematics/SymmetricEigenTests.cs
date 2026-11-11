using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Mathematics;

public class SymmetricEigenTests
{
    [Fact]
    public void DecompositionReconstructsTheMatrix()
    {
        double[] a = [4, 2, 0.4, 2, 5, 1, 0.4, 1, 3];
        var values = new double[3];
        var vectors = new double[9];

        SymmetricEigen.Decompose(a, 3, values, vectors);

        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                // V·diag(λ)·Vᵀ = A, and Vᵀ·V = I.
                double rebuilt = 0, gram = 0;
                for (var k = 0; k < 3; k++)
                {
                    rebuilt += vectors[(i * 3) + k] * values[k] * vectors[(j * 3) + k];
                    gram += vectors[(k * 3) + i] * vectors[(k * 3) + j];
                }

                Assert.Equal(a[(i * 3) + j], rebuilt, 13);
                Assert.Equal(i == j ? 1.0 : 0.0, gram, 14);
            }
        }

        // Trace and determinant: 12 and 4·(15−1) − 2·(6−0.4) + 0.4·(2−2) = 44.8.
        Assert.Equal(12.0, values.Sum(), 13);
        Assert.Equal(44.8, values[0] * values[1] * values[2], 12);
    }

    [Fact]
    public void ReadsOnlyTheLowerTriangle()
    {
        double[] a = [2, 99, 1, 2];
        var values = new double[2];
        var vectors = new double[4];

        SymmetricEigen.Decompose(a, 2, values, vectors);

        Array.Sort(values);
        Assert.Equal(1.0, values[0], 14);
        Assert.Equal(3.0, values[1], 14);
    }

    [Fact]
    public void MinimumNormSolveMatchesCholeskyWhenNonSingular()
    {
        double[] a = [4, 2, 0.4, 2, 5, 1, 0.4, 1, 3];
        double[] b = [0.2, -7.5, -0.1]; // A·[1, −2, 0.5].

        var rank = SymmetricEigen.SolveMinimumNorm(a, 3, b);

        Assert.Equal(3, rank);
        Assert.Equal(1.0, b[0], 13);
        Assert.Equal(-2.0, b[1], 13);
        Assert.Equal(0.5, b[2], 13);
    }

    [Theory]
    [InlineData(2.0, 2.0, 1.0, 1.0)] // Consistent: the smallest of x0 + x1 = 2.
    [InlineData(2.0, 0.0, 0.5, 0.5)] // Inconsistent: least squares, then smallest.
    public void SingularSystemsGetTheSmallestLeastSquaresSolution(double b0, double b1, double x0, double x1)
    {
        double[] a = [1, 1, 1, 1];
        double[] b = [b0, b1];

        var rank = SymmetricEigen.SolveMinimumNorm(a, 2, b);

        Assert.Equal(1, rank);
        Assert.Equal(x0, b[0], 14);
        Assert.Equal(x1, b[1], 14);
    }

    [Fact]
    public void ZeroMatrixHasRankZeroAndAZeroSolution()
    {
        double[] b = [3, -1];

        Assert.Equal(0, SymmetricEigen.SolveMinimumNorm(new double[4], 2, b));
        Assert.Equal([0.0, 0.0], b);
    }

    [Fact]
    public void RejectsWrongShapes()
    {
        Assert.Throws<ArgumentException>(() => SymmetricEigen.Decompose(new double[5], 2, new double[2], new double[4]));
        Assert.Throws<ArgumentException>(() => SymmetricEigen.Decompose(new double[4], 2, new double[3], new double[4]));
        Assert.Throws<ArgumentException>(() => SymmetricEigen.SolveMinimumNorm(new double[4], 2, new double[3]));
    }
}
