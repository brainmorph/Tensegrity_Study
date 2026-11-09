using Tensegrity.Core.Bodies;
using Tensegrity.Core.Mathematics;

namespace Tensegrity.Core.Tests.Bodies;

public class PlanarLinkTests
{
    [Fact]
    public void SlenderBarInertiaIsMassLengthSquaredOverTwelve()
    {
        var link = PlanarLink.SlenderBar("bar", mass: 3.0, length: 2.0);

        Assert.Equal(1.0, link.MomentOfInertia, 15);
        Assert.Equal(new Vector2d(-1, 0), link.EndA);
        Assert.Equal(new Vector2d(1, 0), link.EndB);
    }

    [Fact]
    public void InertiaAboutEndIsMassLengthSquaredOverThree()
    {
        var link = PlanarLink.SlenderBar("bar", mass: 3.0, length: 2.0);

        Assert.Equal(4.0, link.MomentOfInertiaAbout(link.EndA), 14);
    }

    [Theory]
    [InlineData(0.0, 1.0, 1.0)]
    [InlineData(-1.0, 1.0, 1.0)]
    [InlineData(double.NaN, 1.0, 1.0)]
    [InlineData(1.0, 0.0, 1.0)]
    [InlineData(1.0, 1.0, -1.0)]
    [InlineData(1.0, 1.0, double.PositiveInfinity)]
    public void RejectsNonPhysicalParameters(double mass, double inertia, double length)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new PlanarLink("bar", mass, inertia, length));
    }
}
