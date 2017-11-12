/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

import java.io.Serializable;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IBodyPartConfiguration extends Serializable {
	/**
	 * Returns the name of the body part.
	 *
	 * @return the name of the body part
	 */
	String getName();

	/**
	 * Returns the name of the parent body part.
	 *
	 * @return the name of the parent body part - or null if there is no parent
	 *         body part.
	 */
	String getParent();

	/**
	 * Returns a vector that describes the position of the body part relative to
	 * it's parent.
	 *
	 * @return the body translation (meter) - or null if there is no parent body
	 *         part.
	 */
	Vector3D getTranslation();

	/**
	 * Returns the mass of the body part.
	 *
	 * @return the body mass (kg)
	 */
	float getMass();

	/**
	 * Returns a vector, containing the geometry of the body part.
	 *
	 * @return the body geometry (meters)
	 */
	Vector3D getGeometry();

	/**
	 * Returns the JointConfiguration object to that body part.
	 *
	 * @return the joint configuration - or null if there is no joint attached
	 */
	ISensorConfiguration getJointConfiguration();

	/**
	 * Returns the anchor of the joint, relative to the body part.
	 *
	 * @return the anchor of the joint - or null if there is no joint attached
	 */
	Vector3D getJointAnchor();

	/**
	 * Returns the configuration of the GyroRate perceptor inside the body part.
	 *
	 * @return the configuration of the gyro-rate perceptor - or null if the body
	 *         contains no such perceptor
	 */
	ISensorConfiguration getGyroRateConfiguration();

	/**
	 * Returns the configuration of the Accelerometer inside the body part.
	 *
	 * @return the configuration of the accelerometer - or null if the body
	 *         contains no such perceptor
	 */
	ISensorConfiguration getAccelerometerConfiguration();

	/**
	 * Returns the configuration of the ForceResistance perceptor inside the body
	 * part.
	 *
	 * @return the configuration of the force-resistance perceptor - or null if
	 *         the body contains no such perceptor
	 */
	ISensorConfiguration getForceResistanceConfiguration();

	/**
	 * Returns the configuration of the CompassConfig perceptor inside the body
	 * part.
	 *
	 * @return the configuration of the force-resistance perceptor - or null if
	 *         the body contains no such perceptor
	 */
	ISensorConfiguration getCompassConfig();
}
