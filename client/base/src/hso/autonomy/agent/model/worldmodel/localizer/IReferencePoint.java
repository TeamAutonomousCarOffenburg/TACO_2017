/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Reference point that can be used for localization.
 *
 * @author Stefan Glaser
 *
 */
public interface IReferencePoint extends Comparable<IReferencePoint> {
	/**
	 * @return the observed position under which we see the reference point
	 *         (relative to the root body)
	 */
	Vector3D getLocalPosition();

	/**
	 * @return the position at which we know the reference point is located
	 */
	Vector3D getKnownPosition();

	/**
	 * @return indicates, if the reference point is currently visible (position
	 *         is up to date)
	 */
	boolean isVisible();
}