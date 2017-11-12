package taco.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import taco.agent.communication.perception.IImuPerceptor;
import taco.agent.communication.perception.PerceptorName;

public class ImuPerceptor extends AudiCupPerceptor implements IImuPerceptor
{
	private Rotation gyro;

	private Vector3D acceleration;

	public ImuPerceptor(long timestamp, Rotation gyro, Vector3D acceleration)
	{
		super(PerceptorName.CAR_IMU, timestamp);
		this.gyro = gyro;
		this.acceleration = acceleration;
	}

	@Override
	public Rotation getGyro()
	{
		return gyro;
	}

	@Override
	public Vector3D getAcceleration()
	{
		return acceleration;
	}
}
