package taco.agent.model.agentmodel.impl;

import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmodel.IAudiCupActuator;

/**
 * Base class for all actuators (motor, steering, light, ...)
 */
public abstract class AudiCupActuator implements IAudiCupActuator
{
	/** The name of the actuator. */
	private String name;

	/** the global time at which this actuator was last set */
	protected float modificationTime = -1;

	/** the current global time */
	protected float currentTime;

	public AudiCupActuator(String name)
	{
		this.name = name;
	}

	public boolean update(IAudiCupPerception perception)
	{
		if (perception.getTime() != null) {
			currentTime = perception.getTime().getTime();
		}
		return false;
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public boolean isInitialized()
	{
		return true;
	}

	@Override
	public float getModificationTime()
	{
		return modificationTime;
	}
}
