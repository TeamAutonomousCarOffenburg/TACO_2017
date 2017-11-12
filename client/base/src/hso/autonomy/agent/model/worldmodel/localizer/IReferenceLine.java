/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Reference line that can be used for localization.
 *
 * @author Stefan Glaser
 *
 */
public interface IReferenceLine {
	/**
	 * Update the local position of the reference points of the line (relative to
	 * the root body)<br>
	 * Note: By updating a reference line, the line get's visible
	 * @param newLocalPos1 the new local position under which we see the
	 *        reference point 1
	 * @param newLocalPos2 the new local position under which we see the
	 *        reference point 2
	 */
	void updateLocalPositions(Vector3D newLocalPos1, Vector3D newLocalPos2);

	/**
	 * @return the observed position under which we see the first reference point
	 *         of the line (relative to the root body)
	 */
	Vector3D getLocalPosition1();

	/**
	 * @return the observed position under which we see the second reference
	 *         point of the line (relative to the root body)
	 */
	Vector3D getLocalPosition2();

	/**
	 * @return the position at which we know the first reference point is located
	 */
	Vector3D getKnownPosition1();

	/**
	 * @return the position at which we know the second reference point is
	 *         located
	 */
	Vector3D getKnownPosition2();

	/**
	 * This method returns the same as the getKnownPosition1() in case if the
	 * line is fully visible. Is the line not fully visible, this method returns
	 * an adjusted known position according to the length of the seen line.
	 * @return the known position of the second reference point after an possible
	 *         adjustment, if the line is not fully visible
	 */
	Vector3D getAdjustedKnownPosition1();

	/**
	 * This method returns the same as the getKnownPosition2() in case if the
	 * line is fully visible. Is the line not fully visible, this method returns
	 * an adjusted known position according to the length of the seen line.
	 * @return the known position of the second reference point after an possible
	 *         adjustment, if the line is not fully visible
	 */
	Vector3D getAdjustedKnownPosition2();

	/**
	 * Sets the adjusted known position to the first reference point
	 */
	void adjustKnownPosition1(Vector3D adjustedKnwonPosition1);

	/**
	 * Sets the adjusted known position to the second reference point
	 */
	void adjustKnownPosition2(Vector3D adjustedKnwonPosition2);

	/**
	 * Set visibility of reference line (true = assigned; false = not assigned)
	 */
	void setVisible(boolean isVisible);

	/**
	 * @return indicates, if the reference line is currently visible (assigned)
	 */
	boolean isVisible();
}
