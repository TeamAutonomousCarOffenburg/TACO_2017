/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IAccelerometerPerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IAccelerometer;
import hso.autonomy.agent.model.agentmodel.ISensor;

/**
 * Implementation of an accelerometer sensor
 *
 * @author Ingo Schindler
 */
public class Accelerometer extends Sensor implements IAccelerometer
{
	private Vector3D acceleration;

	/**
	 * Instantiates a new Accelerometer sensor
	 *
	 * @param name Sensor Name
	 */
	public Accelerometer(String name, String perceptorName)
	{
		super(name, perceptorName);
		acceleration = Vector3D.ZERO;
	}

	/**
	 * Copy constructor
	 * @param source the object to copy from
	 */
	private Accelerometer(Accelerometer source)
	{
		super(source.name, source.perceptorName);
		acceleration = source.acceleration;
	}

	@Override
	public Vector3D getAcceleration()
	{
		return acceleration;
	}

	@Override
	public String getName()
	{
		return name;
	}

	/**
	 * Set 3-dimensional accelerations
	 *
	 * @param value Accelerations
	 */
	void setAcceleration(Vector3D value)
	{
		double x = Double.isNaN(value.getX()) ? 0 : value.getX();
		double y = Double.isNaN(value.getY()) ? 0 : value.getY();
		double z = Double.isNaN(value.getZ()) ? 0 : value.getZ();

		acceleration = new Vector3D(x, y, z);
	}

	/**
	 * Updates this accelerometer from perception
	 * @param perception the result from server message parsing
	 */
	@Override
	public void updateFromPerception(IPerception perception)
	{
		IAccelerometerPerceptor perceptor = perception.getAccelerationPerceptor(getPerceptorName());
		if (perceptor != null) {
			setAcceleration(perceptor.getAcceleration());
		}
	}

	@Override
	public ISensor copy()
	{
		return new Accelerometer(this);
	}
}
