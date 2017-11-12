/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Accelerometer sensor interface. Provides read-only access only.
 *
 * @author Ingo Schindler
 */
public interface IAccelerometer extends ISensor {
	/**
	 * @return the acceleration in x, y and z direction (units ??)
	 */
	Vector3D getAcceleration();
}
