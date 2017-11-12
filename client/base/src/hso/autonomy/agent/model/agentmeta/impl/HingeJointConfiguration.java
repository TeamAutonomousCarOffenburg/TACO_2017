/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.agentmeta.IHingeJointConfiguration;

public class HingeJointConfiguration implements IHingeJointConfiguration
{
	private final String name;

	private final String perceptorName;

	private final String effectorName;

	private final Vector3D jointAxis;

	private final int minAngle;

	private final int maxAngle;

	private float maxSpeed;

	private float maxAcceleration;

	private float gain;

	private final boolean defaultToInitialPos;

	public HingeJointConfiguration(String name, String perceptorName, String effectorName, Vector3D jointAxis,
			int minAngle, int maxAngle, float maxSpeed, float maxAcceleration, float gain, boolean defaultToInitialPos)
	{
		this.name = name;
		this.perceptorName = perceptorName;
		this.effectorName = effectorName;
		this.jointAxis = jointAxis;
		this.minAngle = minAngle;
		this.maxAngle = maxAngle;
		this.maxSpeed = maxSpeed;
		this.maxAcceleration = maxAcceleration;
		this.gain = gain;
		this.defaultToInitialPos = defaultToInitialPos;
	}

	public HingeJointConfiguration(String name, String perceptorName, String effectorName, Vector3D jointAxis,
			int jointMinAngle, int jointMaxAngle, float jointMaxSpeed, boolean defaultToInitialPos)
	{
		this(name, perceptorName, effectorName, jointAxis, jointMinAngle, jointMaxAngle, jointMaxSpeed, Float.MAX_VALUE,
				0, defaultToInitialPos);
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public String getPerceptorName()
	{
		return perceptorName;
	}

	@Override
	public String getEffectorName()
	{
		return effectorName;
	}

	@Override
	public Vector3D getJointAxis()
	{
		return jointAxis;
	}

	@Override
	public int getMinAngle()
	{
		return minAngle;
	}

	@Override
	public int getMaxAngle()
	{
		return maxAngle;
	}

	@Override
	public float getMaxSpeed()
	{
		return maxSpeed;
	}

	@Override
	public float getMaxAcceleration()
	{
		return maxAcceleration;
	}

	@Override
	public float getGain()
	{
		return gain;
	}

	@Override
	public void setMaxSpeed(float maxSpeed)
	{
		this.maxSpeed = maxSpeed;
	}

	@Override
	public boolean getDefaultToInitialPos()
	{
		return defaultToInitialPos;
	}
}
