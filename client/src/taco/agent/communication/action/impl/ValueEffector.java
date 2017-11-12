package taco.agent.communication.action.impl;

import hso.autonomy.agent.communication.action.impl.Effector;

public class ValueEffector<T> extends Effector
{
	private T value;

	public ValueEffector(String name, T value)
	{
		super(name);
		this.value = value;
	}

	@Override
	public void setEffectorValues(float maxGain, float... values)
	{
	}

	public void setValue(T value)
	{
		this.value = value;
	}

	public T getValue()
	{
		return value;
	}
}
