package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.decision.behavior.IBehaviorConstants;

public class FollowLeftLane extends FollowLaneBase
{
	public FollowLeftLane(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.FOLLOW_LEFT_LANE, thoughtModel, null);
	}

	@Override
	protected int getDeltaX()
	{
		return getWorldModel().getLaneMiddleSensor().getLeftDeltaX();
	}
}
