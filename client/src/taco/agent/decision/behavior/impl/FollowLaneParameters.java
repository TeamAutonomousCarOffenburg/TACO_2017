package taco.agent.decision.behavior.impl;

import kdo.util.parameter.EnumParameterList;

/**
 * Parameters used in following lane
 */
public class FollowLaneParameters extends EnumParameterList<FollowLaneParameters.Param>
{
	public enum Param { INPUT_FACTOR }

	public FollowLaneParameters()
	{
		super(Param.class);
	}

	@Override
	protected void setValues()
	{
		put(Param.INPUT_FACTOR, 0.13f);
	}

	public float getInputFactor()
	{
		return get(Param.INPUT_FACTOR);
	}
}
