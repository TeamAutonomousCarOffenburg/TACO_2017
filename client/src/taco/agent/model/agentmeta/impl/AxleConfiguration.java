package taco.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class AxleConfiguration extends SensorConfiguration
{
	private Vector3D position;

	private float length;

	private float wheelDiameter;

	public AxleConfiguration(String name, String perceptorName)
	{
		super(name, perceptorName);
	}

	public Vector3D getPosition()
	{
		return position;
	}

	public float getLength()
	{
		return length;
	}

	public float getWheelDiameter()
	{
		return wheelDiameter;
	}
}
