/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

/**
 * "CompositeJoint" sensor interface.
 *
 * @author Stefan Glaser, Klaus Dorer
 */
public interface ICompositeJoint extends IJoint {
	/**
	 * @param index index of the joint to work on
	 * @return the minimal angle that can be reached by joint (in degrees)
	 */
	float getMinAngle(int index);

	/**
	 * @param index index of the joint to work on
	 * @return the maximal angle that can be reached by joint (in degrees)
	 */
	float getMaxAngle(int index);

	/**
	 * Retrieve joint axis[index] angle
	 *
	 * @param index index of the joint to work on
	 * @return first angle of the joint
	 */
	float getAngle(int index);

	/**
	 * Let the joint perform a movement along axis with the given speed in the
	 * current cycle.
	 *
	 * @param index index of the joint to work on
	 * @param speed to move the joint axis (in deg/cycle)
	 */
	void performAxisSpeed(int index, float speed);

	/**
	 * Let the joint perform a movement along axis in direction of the given
	 * target position (at maximum speed)
	 *
	 * @param index index of the joint to work on
	 * @param position - target position of axis1 movement
	 */
	void performAxisPosition(int index, float position);

	/**
	 * Let the hinge joint perform a movement along axis in direction of the
	 * given target position with the given speed as maximum speed
	 *
	 * @param index index of the joint to work on
	 * @param position - target position of axis movement
	 * @param maxSpeed - maximum allowed speed for movement (in deg/cycle)
	 */
	void performAxisPosition(int index, float position, float maxSpeed);

	/**
	 * Returns the next axis speed to the joint.<br>
	 * This method is called by the coordinator, during reflecting the future
	 * actions out of the agent model.
	 *
	 * @param index index of the joint to work on
	 * @return next axis1 speed to perform (in deg/cycle)
	 */
	float getNextAxisSpeed(int index);
}