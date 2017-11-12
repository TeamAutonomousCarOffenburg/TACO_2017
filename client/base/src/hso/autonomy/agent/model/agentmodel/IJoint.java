/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

import java.io.Serializable;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

/**
 * @author Stefan Glaser
 *
 */
public interface IJoint extends ISensor, Serializable {
	/**
	 * Called to perform a movement towards the initial position of the joint.
	 */
	void performInitialPosition();

	/**
	 * Called to reset all currently set movements of the joint and perform no
	 * movement at all (zero axis speed).
	 */
	void resetMovement();

	/**
	 * Retrieve the rotation matrix, related to the current axis positions of the
	 * joint.
	 *
	 * @return the rotation matrix of the joint
	 */
	Rotation getRotation();

	/**
	 * Informs the action component to create action commands from this joint.
	 * @param actions a map of effector name, values pairs
	 */
	void generateJointAction(Map<String, float[]> actions);

	/**
	 * Updates the speed values of this joint by copying them from the passed
	 * joint
	 * @param joint the joint used as source
	 */
	void updateJointPositionFromJoint(IJoint joint);

	/**
	 * @return a list of joints that are sub joints. Joints that do not have
	 *         subjoints would return themselves.
	 */
	List<IJoint> getAllSubJoints();
}
