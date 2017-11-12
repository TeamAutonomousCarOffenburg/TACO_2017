/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Unlabeled reference line that can be used for localization.
 *
 * @author Stefan Glaser
 *
 */
public interface ILocalizationLine {
	/**
	 * @return the first observed position under which we see the line (relative
	 *         to the root body)
	 */
	Vector3D getLocalPosition1();

	/**
	 * @return the second observed position under which we see the line (relative
	 *         to the root body)
	 */
	Vector3D getLocalPosition2();
}
