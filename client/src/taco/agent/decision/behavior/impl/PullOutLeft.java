package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.impl.enums.LightName;

public class PullOutLeft extends PullOutBase
{
	public PullOutLeft(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.PULL_OUT_LEFT, thoughtModel);
	}

	@Override
	protected void starting(IPose2D currentCarPose, IAudiCupAgentModel agentModel)
	{
		super.starting(currentCarPose, agentModel);
		agentModel.getLight(LightName.INDICATOR_LEFT).turnOn();
	}

	@Override
	protected void forward(IPose2D currentCarPose, IAudiCupAgentModel agentModel, double length)
	{
		super.forward(currentCarPose, agentModel, 0.5);
	}

	@Override
	protected void turn(IPose2D currentCarPose, IAudiCupAgentModel agentModel)
	{
		if (currentCarPose.getDeltaAngle(startPose).degrees() < -90) {
			agentModel.getLight(LightName.INDICATOR_LEFT).turnOff();
			phase = Phase.FINISHED;
		}
		agentModel.getSteering().steer(Angle.deg(30));
		agentModel.getMotor().driveForward();
	}
}
