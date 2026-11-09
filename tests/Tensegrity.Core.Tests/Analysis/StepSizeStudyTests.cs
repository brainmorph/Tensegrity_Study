using Tensegrity.Core.Analysis;
using Tensegrity.Topology;

namespace Tensegrity.Core.Tests.Analysis;

/// <summary>Test 14 machinery (PLAN.md section 7): the step-size study itself.</summary>
public class StepSizeStudyTests
{
    [Fact]
    public void PendulumStudyShowsFourthOrderConvergence()
    {
        var model = ReferenceTopologies.Pendulum();

        var study = StepSizeStudy.Run(model.System, model.InitialState, 0.004, 0.4, levels: 4);

        Assert.Equal([0.004, 0.002, 0.001, 0.0005], study.Select(l => l.Dt));
        Assert.Null(study[0].Ratio);
        Assert.Null(study[^1].ChangeToNext);
        Assert.InRange(study[1].Ratio!.Value, 14.0, 18.0);
        Assert.InRange(study[2].Ratio!.Value, 14.0, 18.0);
        Assert.True(study[2].EnergyBalanceError < study[0].EnergyBalanceError);
    }

    [Fact]
    public void RejectsDurationsThatAreNotWholeSampledSteps()
    {
        var model = ReferenceTopologies.Pendulum();

        Assert.Throws<ArgumentException>(() => StepSizeStudy.Run(model.System, model.InitialState, 0.003, 0.4, 3));
        Assert.Throws<ArgumentOutOfRangeException>(() => StepSizeStudy.Run(model.System, model.InitialState, 0.004, 0.4, 1));
    }
}
