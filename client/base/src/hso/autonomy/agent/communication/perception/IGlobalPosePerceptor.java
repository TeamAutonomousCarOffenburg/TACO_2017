/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

import hso.autonomy.util.geometry.Pose3D;

/**
 * The Player Position Perceptor represents a single player as reported by the
 * visual sensor sub-system. Provides read-only access only.
 *
 * @author Simon Raffeiner
 */
public interface IGlobalPosePerceptor {
	/**
	 * Retrieve position and rotation of the torso in global coordinate system
	 *
	 * @return position and rotation of the torso in global coordinate system
	 */
	Pose3D getGlobalPose();
}