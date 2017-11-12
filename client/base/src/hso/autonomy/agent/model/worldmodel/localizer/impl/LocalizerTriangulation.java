/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.Collections;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.Pose2D;
import hso.autonomy.util.geometry.Pose3D;

/**
 * Implementation of the ILocalizer interface that uses triangulation for
 * localization
 * @author Klaus Dorer
 */
public class LocalizerTriangulation extends LocalizerBase
{
	/**
	 * Calculates absolute position and directions by performing triangulation
	 * for all pairs of flags taking the average of all calculations.
	 * @return an array of two Vector3Ds, the first containing the absolute x,y,z
	 *         position of the viewer on the field, the second containing no real
	 *         vector, but the horizontal, latitudal and rotational absolute body
	 *         angles of the viewer
	 */
	@Override
	public Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation rootOrientation)
	{
		List<IReferencePoint> flags = getVisibleReferencePoints(environment);

		if (flags.size() < 2) {
			// there are not 2 visible flags
			return null;
		}

		// sort the flags according to their angle
		Collections.sort(flags);

		// get the two flags with maximum angle difference
		int triangulations = flags.size() * (flags.size() - 1) / 2;
		assert triangulations > 0 : "we need at least one triangulation";

		Pose2D[] results = new Pose2D[triangulations];
		int count = 0;
		for (int i = 0; i < flags.size() - 1; i++) {
			IReferencePoint flag1 = flags.get(i);
			for (int j = i + 1; j < flags.size(); j++) {
				IReferencePoint flag2 = flags.get(j);
				results[count] = triangulate(flag1, flag2);
				count++;
			}
		}

		Pose2D p = Pose2D.average(results);

		return new Pose3D(p.getPosition(), Geometry.createZRotation(p.getAngle().radians()));
	}

	/**
	 * Calculates absolute position and directions from the two flags passed
	 * using triangulation. Absolute means with respect to the game fields
	 * coordinate system.
	 * @param flag1 first visible landmark with known position
	 * @param flag2 second visible landmark with known position (has to be right
	 *        of first)
	 * @return an array of two Vector3Ds, the first containing the absolute x,y,z
	 *         position of the viewer on the field, the second containing no real
	 *         vector, but the horizontal, latitudal and rotational absolute body
	 *         angles of the viewer
	 */
	private Pose2D triangulate(IReferencePoint flag1, IReferencePoint flag2)
	{
		float flag1Direction;
		float flag2Direction;
		double r1, r2;		// the distance of the player to the flags
		double dist, dist2; // the distance (square) of the two flags
		double a;			// distance from one flag to intersection
		// point P3
		double h; // distance from P3 to the intersection
		// Points P1 and P2 of the two circles
		double x, x1, x2, x3;
		double y, y1, y2, y3;
		double ratio;
		Angle horizontalAbsDirection;
		float beta;

		// do the calculations
		flag1Direction = (float) flag1.getLocalPosition().getAlpha();
		flag2Direction = (float) flag2.getLocalPosition().getAlpha();

		r1 = flag1.getLocalPosition().getNorm();
		r2 = flag2.getLocalPosition().getNorm();

		x1 = flag1.getKnownPosition().getX();
		x2 = flag2.getKnownPosition().getX();

		y1 = flag1.getKnownPosition().getY();
		y2 = flag2.getKnownPosition().getY();

		// calculate the square distance of the two flags
		dist2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		dist = Math.sqrt(dist2);
		if (dist > r1 + r2) {
			// the circles would not intersect
			dist = r1 + r2;
			dist2 = dist * dist;
		} else if ((r1 > r2) && (dist + r2 < r1)) {
			// the circles would not intersect
			dist = r1 - r2;
			dist2 = dist * dist;
		} else if ((r2 > r1) && (dist + r1 < r2)) {
			// the circles would not intersect
			dist = r2 - r1;
			dist2 = dist * dist;
		}

		r1 *= r1;
		r2 *= r2;

		// a = (r1^2 - r2^2 + d^2 ) / (2 d)
		// a = distance from flag1 to base point of height line
		a = (r1 - r2 + dist2) / (2.0 * dist);

		// h^2 = r1^2 - a^2
		// h = height of the triangle flag1, flag2 and my position
		h = r1 - a * a;
		if (h < 0.0)
			h = 0.0;
		else
			h = Math.sqrt(h);

		// calculate middle of intersection line
		// P3 = P1 + a ( P2 - P1 ) / d
		x3 = x1 + a * (x2 - x1) / dist;
		y3 = y1 + a * (y2 - y1) / dist;

		// two circles intersect usually in 2 points. Find out which one to
		// select
		if (flag1Direction > flag2Direction) {
			// result x = x3 + h ( y2 - y1 ) / d
			x = x3 + h * (y2 - y1) / dist;
			// result y = y3 - h ( x2 - x1 ) / d
			y = y3 - h * (x2 - x1) / dist;
		} else {
			x = x3 - h * (y2 - y1) / dist;
			y = y3 + h * (x2 - x1) / dist;
		}

		// TODO: add z position calculation
		Vector3D position = new Vector3D(x, y, 0);

		// calculate the absolute direction
		r1 = flag1.getLocalPosition().getNorm();
		ratio = (y1 - y) / r1;
		beta = (float) Math.asin(ratio);
		if (x > x1) {
			horizontalAbsDirection = Angle.ANGLE_180.subtract(beta).subtract(flag1Direction);
		} else {
			horizontalAbsDirection = Angle.rad(beta).subtract(flag1Direction);
		}

		// TODO add vertical direction
		return new Pose2D(position, horizontalAbsDirection);
	}
}
