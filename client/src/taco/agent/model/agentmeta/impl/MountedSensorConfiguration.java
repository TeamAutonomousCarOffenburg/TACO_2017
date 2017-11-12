package taco.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;
import hso.autonomy.util.geometry.Pose3D;

public class MountedSensorConfiguration extends SensorConfiguration
{
	private Pose3D pose;

	public MountedSensorConfiguration(String name, String perceptorName, Pose3D pose)
	{
		super(name, perceptorName);
		this.pose = pose;
	}

	public Pose3D getPose()
	{
		return pose;
	}
}
