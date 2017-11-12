package taco.agent.communication.perception.impl;

import taco.agent.communication.perception.IWheelTickPerceptor;

public class WheelTickPerceptor extends AudiCupPerceptor implements IWheelTickPerceptor
{
	private long ticks;

	private int direction;

	public WheelTickPerceptor(String name, long timestamp, long ticks, int direction)
	{
		super(name, timestamp);
		this.ticks = ticks;
		this.direction = direction;
	}

	@Override
	public long getTicks()
	{
		return ticks;
	}

	@Override
	public int getDirection()
	{
		return direction;
	}
}
