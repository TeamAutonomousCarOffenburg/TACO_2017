package taco.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;
import hso.autonomy.util.geometry.Angle;

public class ServoDriveConfiguration extends SensorConfiguration
{
	private float min;

	private float max;

	public ServoDriveConfiguration(String name, String perceptorName)
	{
		super(name, perceptorName);
	}

	/**
	 * Constructor for unit testing
	 */
	public ServoDriveConfiguration(String name, float min, float max)
	{
		super(name, name);
		this.min = min;
		this.max = max;
	}

	public Angle getMin()
	{
		return Angle.deg(min);
	}

	public Angle getMax()
	{
		return Angle.deg(max);
	}
}
