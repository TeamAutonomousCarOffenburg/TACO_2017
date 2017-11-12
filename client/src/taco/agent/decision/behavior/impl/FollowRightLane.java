package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import kdo.util.parameter.ParameterMap;
import taco.agent.decision.behavior.IBehaviorConstants;

public class FollowRightLane extends FollowLaneBase
{
	public FollowRightLane(IThoughtModel thoughtModel, ParameterMap params)
	{
		this(IBehaviorConstants.FOLLOW_RIGHT_LANE, thoughtModel, params);
	}

	public FollowRightLane(String name, IThoughtModel thoughtModel, ParameterMap params)
	{
		super(name, thoughtModel, (params != null) ? (FollowLaneParameters) params.get(name) : null);
	}

	@Override
	protected int getDeltaX()
	{
		return getWorldModel().getLaneMiddleSensor().getDeltaX();
	}
}
