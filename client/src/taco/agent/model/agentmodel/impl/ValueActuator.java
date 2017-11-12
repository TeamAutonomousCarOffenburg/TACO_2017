package taco.agent.model.agentmodel.impl;

/**
 * Actuator base for all actuators that require a single value as state.
 */
public abstract class ValueActuator<T> extends AudiCupActuator
{
	/** the intended value of this actuator */
	protected T value;

	/** the value of this actuator in the last cycle */
	protected T previousValue;

	/**
	 * @param name the specific name of this actuator
	 */
	public ValueActuator(String name)
	{
		super(name);
	}

	protected boolean hasChanged()
	{
		return !previousValue.equals(value);
	}

	protected void setValue(T value)
	{
		if (currentTime != modificationTime) {
			// the set method is called the first time for this object during this cycle
			previousValue = this.value;
			modificationTime = currentTime;
		}
		this.value = value;
	}
}
