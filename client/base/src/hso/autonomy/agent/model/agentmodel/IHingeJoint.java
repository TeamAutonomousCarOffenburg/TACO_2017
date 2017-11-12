/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

/**
 * "HingeJoint" sensor interface.
 *
 * @author Stefan Glaser, Klaus Dorer
 */
public interface IHingeJoint extends IHingeJointR {
	/**
	 * Let the hinge joint perform a movement with the given speed in the current
	 * cycle.
	 *
	 * @param speed to move the joint axis (in deg/cycle)
	 */
	void performAxisSpeed(float speed);

	/**
	 * Let the hinge joint perform a movement with the given speed in the current
	 * cycle.
	 *
	 * @param speed to move the joint axis (in deg/cycle)
	 * @param maxSpeed - maximum allowed speed for movement (in deg/cycle)
	 */
	void performAxisSpeed(float speed, float maxSpeed);

	/**
	 * Let the hinge joint perform a movement in direction of the given target
	 * position (at maximum speed)
	 *
	 * @param position - target position of movement
	 * @return the speed with which we adjust
	 */
	float performAxisPosition(double position);

	/**
	 * Let the hinge joint perform a movement in direction of the given target
	 * position with the given speed as maximum speed
	 *
	 * @param position - target position of movement
	 * @param maxSpeed - maximum allowed speed for movement (in deg/cycle)
	 */
	float performAxisPosition(double position, float maxSpeed);

	/**
	 * Adjusts the desired speed of this joint to reach current position + delta
	 * @param delta the delta angle (deg) to add to the current position
	 */
	void adjustAxisPosition(double delta);

	/**
	 * @return the motor related to this joint
	 */
	IMotor getMotor();

	/**
	 * Allows to directly set desired joint angles, speed and acceleration
	 * @param desiredAngle the angle we want to have in the joint (in degrees)
	 * @param speedAtDesiredAngle the angular speed we want to have at that angle
	 *        (in deg / cycle)
	 * @param accelerationAtDesiredAngle the angular acceleration we want to have
	 *        at angle ax (in deg / cycle^2)
	 */
	void setFutureValues(float desiredAngle, float speedAtDesiredAngle, float accelerationAtDesiredAngle);
}