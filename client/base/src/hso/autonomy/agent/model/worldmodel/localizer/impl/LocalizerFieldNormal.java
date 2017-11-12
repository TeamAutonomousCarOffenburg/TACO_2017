/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * This localization method first calculates a normal to the field ground based
 * on all unlabeled reference lines and reference points on ground level, before
 * it determines the z rotation by relative angle difference of two flags. The
 * resulting orientation is then used together with the reference points to
 * calculate the position.
 *
 * @author Stefan Glaser
 *
 */
public class LocalizerFieldNormal extends LocalizerBase
{
	@Override
	public Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		List<IReferencePoint> refPoints = getVisibleReferencePoints(environment);
		List<IReferenceLine> refLines = getVisibleReferenceLines(environment);

		// We wanna see at least 3 lines and 2 flags to rely on this localization
		// Though, the theoretical minimum amount of lines is 2
		if (lines == null || lines.size() < 3 || refPoints.size() < 2) {
			return null;
		}

		List<Vector3D> bottomPoints = new ArrayList<>();
		Vector3D temp;

		// Add fixpoints on ground level to list of bottom-points
		for (IReferencePoint flag : refPoints) {
			if (FuzzyCompare.eq(0, flag.getKnownPosition().getZ(), 0.001)) {
				bottomPoints.add(flag.getLocalPosition());
			}
		}

		// Add all seen lines to list of bottom-points
		for (ILocalizationLine line : lines) {
			temp = line.getLocalPosition1();
			if (temp != null) {
				bottomPoints.add(temp);
			}
			temp = line.getLocalPosition2();
			if (temp != null) {
				bottomPoints.add(temp);
			}
		}

		// Calculate field normal
		Vector3D normal = LocalizerUtil.calculatePlaneNormal(bottomPoints);
		// Create rotation based on field normal. Since the field normal is the
		// global z-axis, simply create a rotation that transforms the normal back
		// to the global z-axis.
		Rotation normalRotation = new Rotation(normal, Vector3D.PLUS_K);

		// Calculate z rotation based on normalized local positions of the
		// reference points. Since we still have no clue about our position, we
		// can't just simply use the angle difference to one reference point.
		// Instead we have to use an independent calculation of the angles between
		// two reference points in the local and known system to get the z
		// rotation.
		IReferencePoint flag1;
		IReferencePoint flag2;
		Vector3D flag1Pos;
		Vector3D flag1KnownPos;
		Vector3D flag2Pos;
		Vector3D flag2KnownPos;
		Vector3D vec1to2_3d;
		Vector3D vec1to2Known_3d;
		List<Angle> angles = new ArrayList<>();

		// Instead of just using the reference points to determine the z rotation,
		// the assigned lines also contribute to a more precise result.
		// Theoretically just two unlabeled lines and {at least one assigned line
		// or tow reference points} are required to result in a complete
		// orientation estimation.
		// Therefore: Add reference points for all proper positions of each
		// assigned line to the list of reference points
		if (refLines != null) {
			for (IReferenceLine line : refLines) {
				if (line.getLocalPosition1() != null) {
					refPoints.add(new ReferencePoint(line.getLocalPosition1(), line.getAdjustedKnownPosition1()));
				}

				if (line.getLocalPosition2() != null) {
					refPoints.add(new ReferencePoint(line.getLocalPosition2(), line.getAdjustedKnownPosition2()));
				}
			}
		}

		for (int i = 0; i < refPoints.size() - 1; i++) {
			flag1 = refPoints.get(i);
			flag1KnownPos = flag1.getKnownPosition();
			flag1Pos = normalRotation.applyTo(flag1.getLocalPosition());

			for (int j = i + 1; j < refPoints.size(); j++) {
				flag2 = refPoints.get(j);
				temp = flag2.getKnownPosition();
				// Check if the second reference point is at the same known position
				// as the first reference point (can happen, since we also use
				// reference lines). In this case, the selected reference point
				// combination can't provide a rotation estimation.
				if (FuzzyCompare.eq(temp, flag1.getKnownPosition(), 0.01)) {
					continue;
				}

				flag2KnownPos = temp;
				flag2Pos = normalRotation.applyTo(flag2.getLocalPosition());

				vec1to2Known_3d = flag2KnownPos.subtract(flag1KnownPos);
				vec1to2_3d = flag2Pos.subtract(flag1Pos);

				Angle knownDirection = Angle.rad(Math.atan2(vec1to2Known_3d.getY(), vec1to2Known_3d.getX()));
				Angle visionDirection = Angle.rad(Math.atan2(vec1to2_3d.getY(), vec1to2_3d.getX()));

				angles.add(knownDirection.subtract(visionDirection));
			}
		}

		// Add the new calculated z rotation to the previous field normal
		// orientation to get the final agent orientation
		Angle[] anglesArray = angles.toArray(new Angle[angles.size()]);
		double averageAngle = Angle.average(anglesArray).radians();
		Rotation zRotation = Geometry.createZRotation(averageAngle);

		Rotation rotation = zRotation.applyTo(normalRotation);

		// Calculate position based on the calculated orientation and the
		// reference points
		return new Pose3D(LocalizerUtil.calculatePosition(refPoints, rotation), rotation);
	}
}
