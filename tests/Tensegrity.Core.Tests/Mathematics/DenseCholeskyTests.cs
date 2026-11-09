using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Mathematics;

public class DenseCholeskyTests
{
    [Fact]
    public void SolvesSymmetricPositiveDefiniteSystem()
    {
        // A = [[4, 2, 0.4], [2, 5, 1], [0.4, 1, 3]], x = [1, −2, 0.5], b = A·x.
        double[] a = [4, 2, 0.4, 2, 5, 1, 0.4, 1, 3];
        double[] x = [1, -2, 0.5];
        var b = new double[3];
        for (var i = 0; i < 3; i++)
        {
            for (var k = 0; k < 3; k++)
            {
                b[i] += a[(i * 3) + k] * x[k];
            }
        }

        Assert.True(DenseCholesky.TryFactor(a, 3));
        DenseCholesky.Solve(a, 3, b);

        for (var i = 0; i < 3; i++)
        {
            Assert.Equal(x[i], b[i], 14);
        }
    }

    [Fact]
    public void FactorReadsOnlyTheLowerTriangle()
    {
        // Garbage above the diagonal must not matter.
        double[] a = [4, 99, 99, 2, 5, 99, 0.4, 1, 3];
        double[] b = [0.2, -7.5, -0.1]; // A·[1, −2, 0.5] with the true A.

        Assert.True(DenseCholesky.TryFactor(a, 3));
        DenseCholesky.Solve(a, 3, b);

        Assert.Equal(1.0, b[0], 14);
        Assert.Equal(-2.0, b[1], 14);
        Assert.Equal(0.5, b[2], 14);
    }

    [Fact]
    public void RejectsSingularMatrix()
    {
        // Rank 1: two identical rows, as from a duplicated constraint.
        double[] a = [1, 1, 1, 1];

        Assert.False(DenseCholesky.TryFactor(a, 2));
    }

    [Fact]
    public void RejectsIndefiniteMatrix()
    {
        double[] a = [1, 0, 0, -1];

        Assert.False(DenseCholesky.TryFactor(a, 2));
    }

    [Fact]
    public void RejectsWrongShapes()
    {
        Assert.Throws<ArgumentException>(() => DenseCholesky.TryFactor(new double[5], 2));
        Assert.Throws<ArgumentException>(() => DenseCholesky.Solve(new double[4], 2, new double[3]));
    }
}
