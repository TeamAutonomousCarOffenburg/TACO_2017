/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * The Accelerometer perceptor measures acceleration in the three-dimensional
 * space.
 *
 * @author Simon Raffeiner
 */
public interface IAccelerometerPerceptor extends IPerceptor {
	/**
	 * Get acceleration values in the form of a Vector3D object
	 */
	Vector3D getAcceleration();
}