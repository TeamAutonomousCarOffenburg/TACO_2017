package taco.agent.communication.perception.impl;

import taco.agent.communication.perception.IDoublePerceptor;

public class DoublePerceptor extends ValuePerceptor<Double> implements IDoublePerceptor
{
	public DoublePerceptor(String name, long timestamp, Double value)
	{
		super(name, timestamp, value);
	}
}
