/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IHingeJointConfiguration extends ISensorConfiguration {
	/**
	 * Returns a vector, which defines the rotation axis of the joint.
	 *
	 * @return the rotation axis
	 */
	Vector3D getJointAxis();

	/**
	 * Returns the minimum possible position-angle of the joint.
	 *
	 * @return the minimum position angle (deg)
	 */
	int getMinAngle();

	/**
	 * Returns the maximum possible position-angle of the joint.
	 *
	 * @return the maximum position angle (deg)
	 */
	int getMaxAngle();

	/**
	 * Returns the maximum possible speed to the joint.
	 *
	 * @return the maximum possible speed (deg/cycle)
	 */
	float getMaxSpeed();

	/**
	 * Changes the maximal speed of the joint
	 * @param value the maximum possible speed (deg/cycle)
	 */
	void setMaxSpeed(float value);

	/**
	 * Returns the maximum possible acceleration.
	 *
	 * @return the maximum possible speed (deg/cycle//cycle)
	 */
	float getMaxAcceleration();

	/**
	 * @return the name of this effector
	 */
	String getEffectorName();

	/**
	 * @return the stiffness or gain of the joint motor
	 */
	float getGain();

	/**
	 * @return whether to move to initial position automatically if unused this
	 *         cycle
	 */
	boolean getDefaultToInitialPos();
}
