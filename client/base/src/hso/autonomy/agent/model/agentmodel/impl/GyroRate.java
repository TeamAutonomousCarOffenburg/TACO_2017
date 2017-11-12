/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IGyroPerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IGyroRate;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Implementation of a "GyroRate" sensor.
 *
 * @author Stefan Glaser, Srinivasa Ragavan
 */
public class GyroRate extends Sensor implements IGyroRate
{
	/** angular change to previous orientation */
	private Vector3D gyro;

	/**
	 * Instantiates a new GyroRate sensor and initializes all values to their
	 * default
	 *
	 * @param name sensor name
	 */
	public GyroRate(String name, String perceptorName)
	{
		super(name, perceptorName);
		gyro = Vector3D.ZERO;
	}

	/**
	 * Copy constructor
	 * @param source the object to copy from
	 */
	public GyroRate(GyroRate source)
	{
		super(source.name, source.perceptorName);
		gyro = source.gyro;
	}

	@Override
	public Vector3D getGyro()
	{
		return gyro;
	}

	/**
	 * Set 3-dimensional gyro values
	 *
	 * @param gyro Gyro values
	 */
	public void setGyro(Vector3D gyro)
	{
		// Save current rotation-vector
		this.gyro = gyro;
	}

	@Override
	public Rotation getOrientationChange(float cycleTime)
	{
		double x, y, z;

		// Extract the angles of the given rotation-speeds
		x = Math.toRadians(gyro.getX() * cycleTime);
		y = Math.toRadians(gyro.getY() * cycleTime);
		z = Math.toRadians(gyro.getZ() * cycleTime);

		return new Rotation(RotationOrder.YXZ, y, x, z);
	}

	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof GyroRate)) {
			return false;
		}
		GyroRate other = (GyroRate) o;
		if (!super.equals(other)) {
			return false;
		}
		return FuzzyCompare.eq(gyro, other.gyro, 0.00001f);
	}

	/**
	 * Updates this Gyro from perception
	 * @param perception the result from server message parsing
	 */
	@Override
	public void updateFromPerception(IPerception perception)
	{
		IGyroPerceptor perceptor = perception.getGyroRatePerceptor(getPerceptorName());
		if (perceptor != null) {
			setGyro(perceptor.getGyro());
		}
	}

	@Override
	public ISensor copy()
	{
		return new GyroRate(this);
	}
}
