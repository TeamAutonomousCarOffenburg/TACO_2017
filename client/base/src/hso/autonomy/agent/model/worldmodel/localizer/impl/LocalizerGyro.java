/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Implements a 3-dimensional localizer based on reference points and the
 * estimated orientation.
 *
 * @author Stefan Glaser
 */
public class LocalizerGyro extends LocalizerBase
{
	@Override
	public Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		if (estimatedOrientation == null) {
			// there is no gyro information available
			return null;
		}

		// fetch reference points and -lines
		List<IReferencePoint> refPoints = getVisibleReferencePoints(environment);
		List<IReferenceLine> refLines = getVisibleReferenceLines(environment);

		if (refPoints.size() == 0) {
			// there is no visible flag
			return null;
		}

		return new Pose3D(
				LocalizerUtil.calculatePosition(refPoints, refLines, estimatedOrientation), estimatedOrientation);
	}

	// ==================================================
	// Below: experimental localizer implementations
	// ==================================================
	/**
	 * This method searches for line pints near by known flags, to extent the
	 * fix-point base for umeyama orientation estimation.
	 */
	private Pose3D lineSearchingUmeyamaLocalization(
			Collection<IReferencePoint> flagsList, List<ILocalizationLine> lines)
	{
		if (flagsList.size() < 3 || (lines != null && lines.size() < 2)) {
			return null;
		}

		List<Vector3D> visionVec = new ArrayList<>();
		List<Vector3D> fixedVec = new ArrayList<>();
		Vector3D fixedCOM;
		Vector3D visionCOM;
		Vector3D temp;

		for (IReferencePoint flag : flagsList) {
			// Add fixedPosition
			fixedVec.add(flag.getKnownPosition());

			// Add localPosition
			visionVec.add(flag.getLocalPosition());
		}

		// Search for lines, ending at fix-points
		for (IReferencePoint flag : flagsList) {
			// Check on corner flag
			if (FuzzyCompare.eq(0, flag.getKnownPosition().getZ(), 0.001)) {
				for (ILocalizationLine line : lines) {
					if (FuzzyCompare.eq(flag.getLocalPosition(), line.getLocalPosition1(), 0.5)) {
						// We found a border line, starting/ending at the fix-point
						fixedVec.add(flag.getKnownPosition());
						visionVec.add(line.getLocalPosition1());
						// System.out.println("found matching line to corner flag");
					}

					if (FuzzyCompare.eq(flag.getLocalPosition(), line.getLocalPosition2(), 0.5)) {
						// We found a border line, starting/ending at the fix-point
						fixedVec.add(flag.getKnownPosition());
						visionVec.add(line.getLocalPosition2());
						// System.out.println("found matching line to corner flag");
					}
				}
			}

			// Check on goal post flag
			if (flag.getKnownPosition().getZ() > 0.1) {
				for (ILocalizationLine line : lines) {
					if (FuzzyCompare.eq(flag.getLocalPosition(), line.getLocalPosition1(), 1.5)) {
						// We found a border line, starting/ending at the fix-point
						temp = flag.getKnownPosition();

						if (temp.getY() > 0) {
							// Upper goal post
							temp = temp.add(new Vector3D(0, 0.9, -0.8));
						} else {
							// Lower goal post
							temp = temp.add(new Vector3D(0, -0.9, -0.8));
						}
						fixedVec.add(temp);

						if (line.getLocalPosition2() != null) {
							if (temp.getX() > 0) {
								temp = temp.add(new Vector3D(-1.8, 0, 0));
							} else {
								temp = temp.add(new Vector3D(1.8, 0, 0));
							}
							fixedVec.add(temp);
						}

						visionVec.add(line.getLocalPosition1());

						if (line.getLocalPosition2() != null) {
							visionVec.add(line.getLocalPosition2());
						}

						// System.out.println("found matching line to goal post");
					}

					if (FuzzyCompare.eq(flag.getLocalPosition(), line.getLocalPosition2(), 1.5)) {
						// We found a border line, starting/ending at the fix-point
						temp = flag.getKnownPosition();

						if (temp.getY() > 0) {
							// Upper goal post
							temp = temp.add(new Vector3D(0, 0.9, -0.8));
						} else {
							// Lower goal post
							temp = temp.add(new Vector3D(0, -0.9, -0.8));
						}
						fixedVec.add(temp);

						if (line.getLocalPosition1() != null) {
							if (temp.getX() > 0) {
								temp = temp.add(new Vector3D(-1.8, 0, 0));
							} else {
								temp = temp.add(new Vector3D(1.8, 0, 0));
							}
							fixedVec.add(temp);
						}

						visionVec.add(line.getLocalPosition2());

						if (line.getLocalPosition1() != null) {
							visionVec.add(line.getLocalPosition1());
						}

						// System.out.println("found matching line to goal post");
					}
				}
			}
		}

		fixedCOM = Vector3D.ZERO;
		for (Vector3D vec : fixedVec) {
			fixedCOM = fixedCOM.add(vec);
		}
		fixedCOM = fixedCOM.scalarMultiply(1.0 / fixedVec.size());

		visionCOM = Vector3D.ZERO;
		for (Vector3D vec : visionVec) {
			visionCOM = visionCOM.add(vec);
		}
		visionCOM = visionCOM.scalarMultiply(1.0 / visionVec.size());

		Vector3D[] fixedVecArr = new Vector3D[visionVec.size()];
		for (int i = 0; i < fixedVec.size(); i++) {
			fixedVecArr[i] = fixedVec.get(i).subtract(fixedCOM);
		}

		Vector3D[] visionVecArr = new Vector3D[visionVec.size()];
		for (int i = 0; i < visionVec.size(); i++) {
			visionVecArr[i] = visionVec.get(i).subtract(visionCOM);
		}

		Rotation orientation = LocalizerUtil.umeyama(visionVecArr, fixedVecArr);

		return new Pose3D(LocalizerUtil.calculatePosition(flagsList, orientation), orientation);
	}
}
