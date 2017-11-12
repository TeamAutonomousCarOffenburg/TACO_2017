package taco.agent.model.agentmodel.impl.enums;

import taco.agent.communication.perception.PerceptorName;

public enum TachometerPosition {
	LEFT(PerceptorName.LEFT_WHEEL_SPEED),
	RIGHT(PerceptorName.RIGHT_WHEEL_SPEED);

	public final String perceptorName;

	TachometerPosition(String perceptorName)
	{
		this.perceptorName = perceptorName;
	}
}
