/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer;

import java.util.List;

/**
 * The model describing the scenario in which the localization should take
 * place. It holds one set for reference points as well as one for reference
 * lines. A reference point/line relates observed positions with known
 * positions.
 *
 * @author Stefan Glaser
 */
public interface IEnvironmentModel {
	/**
	 * Retrieve the list of known reference points.
	 * @return the known reference points
	 */
	List<IReferencePoint> getReferencePoints();

	/**
	 * Retrieve the list of known reference lines.
	 * @return the known reference lines
	 */
	List<IReferenceLine> getReferenceLines();
}
