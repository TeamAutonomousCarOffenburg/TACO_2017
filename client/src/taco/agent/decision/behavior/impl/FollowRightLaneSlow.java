package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import kdo.util.parameter.ParameterMap;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.IAudiCupMotor;

public class FollowRightLaneSlow extends FollowLaneBase
{
	public FollowRightLaneSlow(IThoughtModel thoughtModel, ParameterMap params)
	{
		super(IBehaviorConstants.FOLLOW_RIGHT_LANE_SLOW, thoughtModel,
				(params != null) ? (FollowLaneParameters) params.get(IBehaviorConstants.FOLLOW_RIGHT_LANE_SLOW) : null);
		setSpeed(IAudiCupMotor.LOW_SPEED);
	}

	@Override
	protected int getDeltaX()
	{
		return getWorldModel().getLaneMiddleSensor().getDeltaX();
	}
}
