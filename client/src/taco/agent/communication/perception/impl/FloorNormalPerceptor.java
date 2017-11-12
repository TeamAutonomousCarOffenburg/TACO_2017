package taco.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import taco.agent.communication.perception.IFloorNormalPerceptor;

public class FloorNormalPerceptor extends ValuePerceptor<Vector3D> implements IFloorNormalPerceptor
{
	public FloorNormalPerceptor(String name, long timestamp, Vector3D value)
	{
		super(name, timestamp, value);
	}
}
