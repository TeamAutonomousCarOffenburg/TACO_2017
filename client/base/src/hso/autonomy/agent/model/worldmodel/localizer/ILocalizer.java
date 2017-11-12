/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import hso.autonomy.util.geometry.Pose3D;

/**
 * Implementations are able to calculate the agent's position and orientation
 * from the passed relative information of reference points and lines.<br>
 * Since the observed lines can not be easily assigned to the corresponding
 * known reference lines, one sub-task of the implementations is to try to match
 * these lines and update the corresponding information in the given environment
 * model.
 *
 * @author Stefan Glaser
 *
 */
public interface ILocalizer {
	/**
	 * Determine the absolute position and orientation of the view-port in the 3D
	 * environment.
	 * @param environment a description of the environment, in which the
	 *        localization should take place (set of known reference points and
	 *        lines)
	 * @param lines - A list of currently visible, unlabeled reference lines for
	 *        localization (for line assignment)
	 * @param estimatedOrientation - An approximation of the current global
	 *        orientation (rotation from root body system to global system)
	 *
	 * @return the localized position and orientation
	 */
	Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation estimatedOrientation);

	/**
	 * Assign all visible, unlabeled reference lines (<tt>lines</tt>) to the set
	 * of known reference lines defined by the environment model.
	 * @param environment a description of the environment (set of known
	 *        reference points and lines)
	 * @param localizerInfo determined position and orientation from an earlier
	 *        run of the localizer
	 * @param lines - A list of currently visible, unlabeled reference lines for
	 *        assignment
	 * @param estimatedOrientation - An approximation of the current global
	 *        orientation (rotation from root body system to global system)
	 * @return success
	 */
	boolean assignReferenceLines(IEnvironmentModel environment, Pose3D localizerInfo, List<ILocalizationLine> lines,
			Rotation estimatedOrientation);
}
