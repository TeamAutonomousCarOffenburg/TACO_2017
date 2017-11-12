package taco.agent.communication.perception.impl;

public class ValuePerceptor<T> extends AudiCupPerceptor
{
	private final T value;

	public ValuePerceptor(String name, long timestamp, T value)
	{
		super(name, timestamp);
		this.value = value;
	}

	public T getValue()
	{
		return value;
	}
}
