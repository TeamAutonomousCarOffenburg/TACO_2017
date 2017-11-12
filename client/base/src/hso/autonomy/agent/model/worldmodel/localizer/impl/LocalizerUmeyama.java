/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Pose3D;

/**
 * This localization method applies the umeyama method on the reference points
 * (and reference lines - not yet implemented) to determine the current
 * orientation in the 3D space. This orientation is then used to transform the
 * local positions of the reference points and lines to calculate the position.
 *
 * @author Stefan Glaser
 *
 */
public class LocalizerUmeyama extends LocalizerBase
{
	@Override
	public Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		List<IReferencePoint> refPoints = getVisibleReferencePoints(environment);
		List<IReferenceLine> refLines = getVisibleReferenceLines(environment);

		// We wanna see at least 3 reference points
		if (refPoints.size() < 3) {
			return null;
		}

		Vector3D[] visionVecArr = new Vector3D[refPoints.size()];
		Vector3D[] fixedVecArr = new Vector3D[refPoints.size()];
		Vector3D fixedCOM;
		Vector3D visionCOM;
		Vector3D temp;

		// Transform the global and local positions of each reference point to the
		// body-system and store the resulting vectors in the corresponding
		// arrays.
		int i = 0;
		for (IReferencePoint flag : refPoints) {
			// Add fixedPosition
			fixedVecArr[i] = flag.getKnownPosition();

			// Add localPosition
			visionVecArr[i] = flag.getLocalPosition();
			i++;
		}

		// Transform the global and local positions of each reference line to the
		// body-system and store the resulting vectors in the corresponding
		// arrays.
		if (refLines != null) {
			for (IReferenceLine line : refLines) {
				temp = line.getLocalPosition1();
				if (temp != null) {
					// Add localPosition
					visionVecArr[i] = temp;

					// Add fixedPosition
					fixedVecArr[i] = line.getAdjustedKnownPosition1();
					i++;
				}

				temp = line.getLocalPosition2();
				if (temp != null) {
					// Add localPosition
					visionVecArr[i] = temp;

					// Add fixedPosition
					fixedVecArr[i] = line.getAdjustedKnownPosition2();
					i++;
				}
			}
		}

		// Calculate centroid of fixed points
		fixedCOM = Vector3D.ZERO;
		for (Vector3D vec : fixedVecArr) {
			fixedCOM = fixedCOM.add(vec);
		}
		fixedCOM = fixedCOM.scalarMultiply(1.0 / fixedVecArr.length);

		// Calculate centroid of vision points
		visionCOM = Vector3D.ZERO;
		for (Vector3D vec : visionVecArr) {
			visionCOM = visionCOM.add(vec);
		}
		visionCOM = visionCOM.scalarMultiply(1.0 / visionVecArr.length);

		// Calculate fixed covariance vectors
		for (i = 0; i < fixedVecArr.length; i++) {
			fixedVecArr[i] = fixedVecArr[i].subtract(fixedCOM);
		}
		// Calculate vision covariance vectors
		for (i = 0; i < visionVecArr.length; i++) {
			visionVecArr[i] = visionVecArr[i].subtract(visionCOM);
		}

		// Determine orientation by umeyama
		Rotation orientation = LocalizerUtil.umeyama(visionVecArr, fixedVecArr);

		// TODO: Here is maybe an open problem! The calculated orientation is with
		// respect to the centroid, not to our own position and is therefore not
		// necessarily the same as our orientation!
		// We maybe have to translate the vision vectors before and after the
		// rotation according to the centroid before position estimation and then
		// perform a second umeyama with all known positions adjusted to the
		// localized position.

		// Determine position and return result
		return new Pose3D(LocalizerUtil.calculatePosition(refPoints, refLines, orientation), orientation);
	}
}
