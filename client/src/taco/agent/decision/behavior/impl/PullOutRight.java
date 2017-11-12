package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.impl.enums.LightName;

public class PullOutRight extends PullOutBase
{
	public PullOutRight(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.PULL_OUT_RIGHT, thoughtModel);
	}

	@Override
	protected void starting(IPose2D currentCarPose, IAudiCupAgentModel agentModel)
	{
		super.starting(currentCarPose, agentModel);
		agentModel.getLight(LightName.INDICATOR_RIGHT).turnOn();
	}

	@Override
	protected void turn(IPose2D currentCarPose, IAudiCupAgentModel agentModel)
	{
		if (currentCarPose.getDeltaAngle(startPose).degrees() > 90) {
			agentModel.getLight(LightName.INDICATOR_RIGHT).turnOff();
			phase = Phase.FINISHED;
		}
		agentModel.getSteering().steer(Angle.deg(-30));
		agentModel.getMotor().driveForward();
	}
}
