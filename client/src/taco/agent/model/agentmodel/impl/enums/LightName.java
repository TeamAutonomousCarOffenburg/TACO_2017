package taco.agent.model.agentmodel.impl.enums;

import taco.agent.communication.action.EffectorName;

public enum LightName {
	HEAD(EffectorName.HEAD_LIGHTS),
	BACK(EffectorName.BACK_LIGHTS),
	BRAKE(EffectorName.BRAKE_LIGHTS),
	WARN(EffectorName.WARN_LIGHTS),
	INDICATOR_LEFT(EffectorName.INDICATOR_LIGHTS_LEFT),
	INDICATOR_RIGHT(EffectorName.INDICATOR_LIGHTS_RIGHT);

	public final String effectorName;

	LightName(String effectorName)
	{
		this.effectorName = effectorName;
	}
}
