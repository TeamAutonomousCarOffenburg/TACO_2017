/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action.impl;

/**
 * Implementation of a "HingeJoint" effector, used to move robot joints
 *
 * @author Klaus Dorer
 */
public class HingeEffector extends Effector
{
	/**
	 * the speed of the effector in degrees per cycle - zero means the angle is
	 * not changed
	 */
	private float speed;

	private float lastSpeed;

	/** the desired angle of the effector in degrees */
	private float desiredAngle;

	/**
	 * the speed we want to have when reaching desired angle (in degrees per
	 * cycle)
	 */
	private float speedAtDesiredAngle;

	/**
	 * the acceleration we want to have when reaching desired angle (in degrees
	 * per cycle per cycle)
	 */
	private float accelerationAtDesiredAngle;

	private float gain;

	/**
	 * Instantiates a new HingeJoint effector and initializes all fields to zero
	 *
	 * @param name Hinge Joint name
	 */
	public HingeEffector(String name)
	{
		super(name);

		speed = 0.0f;
		desiredAngle = 0.0f;
		lastSpeed = 0.0f;
		gain = 0.0f;
	}

	@Override
	public void setEffectorValues(float maxGain, float... values)
	{
		this.speed = values[0];
		this.desiredAngle = values[1];
		this.speedAtDesiredAngle = values[2];
		this.accelerationAtDesiredAngle = values[3];
		this.gain = values[4];
		if (gain > maxGain) {
			gain = maxGain;
		}
	}

	public boolean hasChanged()
	{
		return speed != lastSpeed;
	}

	@Override
	public void resetAfterAction()
	{
		lastSpeed = speed;
		speed = 0.0f;
	}

	public float getSpeed()
	{
		return speed;
	}

	/**
	 * @return the desired angular position of this effector in degrees
	 */
	public float getDesiredAngle()
	{
		return desiredAngle;
	}

	public float getGain()
	{
		return gain;
	}

	public float getSpeedAtDesiredAngle()
	{
		return speedAtDesiredAngle;
	}

	public float getAccelerationAtDesiredAngle()
	{
		return accelerationAtDesiredAngle;
	}
}
