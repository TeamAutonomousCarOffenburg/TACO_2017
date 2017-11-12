package taco.agent.model.agentmodel.impl.enums;

import taco.agent.communication.perception.PerceptorName;

public enum UltrasonicPosition {
	FRONT_LEFT(PerceptorName.US_FRONT_LEFT),
	FRONT_CENTER_LEFT(PerceptorName.US_FRONT_CENTER_LEFT),
	FRONT_CENTER(PerceptorName.US_FRONT_CENTER),
	FRONT_CENTER_RIGHT(PerceptorName.US_FRONT_CENTER_RIGHT),
	FRONT_RIGHT(PerceptorName.US_FRONT_RIGHT),

	SIDE_LEFT(PerceptorName.US_SIDE_LEFT),
	SIDE_RIGHT(PerceptorName.US_SIDE_RIGHT),

	REAR_LEFT(PerceptorName.US_REAR_LEFT),
	REAR_CENTER(PerceptorName.US_REAR_CENTER),
	REAR_RIGHT(PerceptorName.US_REAR_RIGHT);

	public final String perceptorName;

	UltrasonicPosition(String perceptorName)
	{
		this.perceptorName = perceptorName;
	}

	public static UltrasonicPosition fromPerceptorName(String perceptorName)
	{
		for (UltrasonicPosition position : values()) {
			if (position.perceptorName.equals(perceptorName)) {
				return position;
			}
		}
		return null;
	}
}
