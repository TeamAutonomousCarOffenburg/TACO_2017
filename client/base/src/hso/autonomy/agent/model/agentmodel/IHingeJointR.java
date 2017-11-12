/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * "HingeJoint" sensor read only interface.
 *
 * @author Stefan Glaser, Klaus Dorer
 */
public interface IHingeJointR extends IJoint {
	/**
	 * @return the minimal angle that can be reached by the joint (in degrees)
	 */
	float getMinAngle();

	/**
	 * @return the maximal angle that can be reached by the joint (in degrees)
	 */
	float getMaxAngle();

	/**
	 * Retrieve the current axis position angle of the joint.
	 *
	 * @return current angle of the joint (in degrees)
	 */
	float getAngle();

	/**
	 * Returns the next axis speed to the joint.<br>
	 * This method is called by the coordinator, during reflecting the future
	 * actions out of the agent model.
	 *
	 * @return next axis speed to perform (in deg/cycle)
	 */
	float getNextAxisSpeed();

	/**
	 * @return the maximum speed this joint can go (in degree per cycle)
	 */
	float getMaxSpeed();

	Vector3D getJointAxis();

	/**
	 * @return the acceleration in deg/cycle//cycle
	 */
	float getAcceleration();
}