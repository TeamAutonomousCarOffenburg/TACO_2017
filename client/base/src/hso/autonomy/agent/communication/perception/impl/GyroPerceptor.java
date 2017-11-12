/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IGyroPerceptor;

/**
 * Gyro perceptor
 *
 * @author Simon Raffeiner
 */
public class GyroPerceptor extends Perceptor implements IGyroPerceptor
{
	private Vector3D vector;

	/**
	 * Default constructor, initializes the gyro to (0, 0, 0)
	 *
	 * @param name Perceptor name
	 */
	public GyroPerceptor(String name)
	{
		this(name, 0.0f, 0.0f, 0.0f);
	}

	public GyroPerceptor(String name, float rotationX, float rotationY, float rotationZ)
	{
		super(name);
		this.vector = new Vector3D(rotationX, rotationY, rotationZ);
	}

	@Override
	public Vector3D getGyro()
	{
		return vector;
	}
}
