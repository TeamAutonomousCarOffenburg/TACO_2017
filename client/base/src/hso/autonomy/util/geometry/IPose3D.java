/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import java.io.Serializable;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Interface for a 3D pose.
 *
 * @author Stefan Glaser
 */
public interface IPose3D extends Serializable {
	/**
	 * Retrieve the x-position value.
	 *
	 * @return the x-position value
	 */
	double getX();

	/**
	 * Retrieve the y-position value.
	 *
	 * @return the y-position value
	 */
	double getY();

	/**
	 * Retrieve the z-position value.
	 *
	 * @return the z-position value
	 */
	double getZ();

	/**
	 * Retrieve the position.
	 *
	 * @return the position
	 */
	Vector3D getPosition();

	/**
	 * Retrieve the orientation.
	 *
	 * @return the orientation
	 */
	Rotation getOrientation();

	/**
	 * Apply this pose as a rigid transformation to another pose. The resulting
	 * pose is stored in a new object.
	 *
	 * @param other the pose to transform
	 * @return a new pose object holding the resulting pose
	 */
	IPose3D applyTo(IPose3D other);

	/**
	 * Apply this pose inversely as a rigid transformation to another pose. The
	 * resulting pose is stored in a new object.
	 *
	 * @param other the pose to transform
	 * @return a new pose object holding the resulting pose
	 */
	IPose3D applyInverseTo(IPose3D other);

	/**
	 * Apply this pose as a rigid transformation to the given position vector.
	 *
	 * @param position the position vector to transform
	 * @return the transformed vector
	 */
	Vector3D applyTo(Vector3D position);

	/**
	 * Apply this pose inversely as a rigid transformation to the given position
	 * vector.
	 *
	 * @param position the position vector to transform
	 * @return the inversely transformed vector
	 */
	Vector3D applyInverseTo(Vector3D position);
}
