package taco.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.Pose3D;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.IImuPerceptor;
import taco.agent.model.agentmodel.IImuSensor;

/**
 * Representation of an integrated IMU sensor.
 */
public class ImuSensor extends AudiCupSensor implements IImuSensor
{
	/** amount and direction of current acceleration */
	private Vector3D acceleration;

	private Rotation gyro;

	/** the current orientation of the sensor with respect to its initial orientation */
	private Rotation orientation;

	private Rotation initialOrientation;

	private boolean initialized;

	public ImuSensor(String name, Pose3D pose)
	{
		super(name, pose);
		gyro = Rotation.IDENTITY;
		orientation = Rotation.IDENTITY;
		initialOrientation = Rotation.IDENTITY;
		initialized = false;
	}

	@Override
	public void updateFromPerception(IPerception perception)
	{
		IImuPerceptor imuPerceptor = ((IAudiCupPerception) perception).getCarImuPerceptor(getPerceptorName());

		if (imuPerceptor != null) {
			Rotation perceptorGyro = imuPerceptor.getGyro();
			double[] angles = perceptorGyro.getAngles(RotationOrder.XYZ);
			// Transformation from left-handed in right-handed coordinates
			perceptorGyro = new Rotation(RotationOrder.XYZ, -angles[0], -angles[1], -angles[2]);

			// Transformation from left-handed in right-handed coordinates and 90deg rotation around Z.
			Vector3D perceptorAcc = imuPerceptor.getAcceleration();
			perceptorAcc = new Vector3D(-perceptorAcc.getY(), perceptorAcc.getX(), -perceptorAcc.getZ());

			acceleration = pose.getOrientation().applyTo(perceptorAcc);
			gyro = pose.getOrientation().applyTo(pose.getOrientation().applyInverseTo(perceptorGyro));
			orientation = initialOrientation.applyTo(gyro);
			lastMeasurementTime = imuPerceptor.getTimeStamp();

			if (!initialized) {
				reset();
				initialized = true;
			}
		}
	}

	@Override
	public Rotation getOrientation()
	{
		return orientation;
	}

	@Override
	public Angle getHorizontalAngle()
	{
		return Geometry.getHorizontalAngle(orientation);
	}

	@Override
	public Vector3D getAcceleration()
	{
		return acceleration;
	}

	@Override
	public boolean isInitialized()
	{
		return initialized;
	}

	public void reset()
	{
		initialOrientation = gyro.revert();
	}
}
