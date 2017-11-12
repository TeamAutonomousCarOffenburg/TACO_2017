package taco.agent.model.agentmeta.impl;

import hso.autonomy.util.geometry.Pose3D;

public class DistanceSensorConfiguration extends MountedSensorConfiguration
{
	private float minDistance;

	private float maxDistance;

	private float scanWidth;

	public DistanceSensorConfiguration(
			String name, String perceptorName, Pose3D pose, float minDistance, float maxDistance, float scanWidth)
	{
		super(name, perceptorName, pose);
		this.minDistance = minDistance;
		this.maxDistance = maxDistance;
		this.scanWidth = scanWidth;
	}

	public float getMinDistance()
	{
		return minDistance;
	}

	public float getMaxDistance()
	{
		return maxDistance;
	}

	public float getScanWidth()
	{
		return scanWidth;
	}
}
