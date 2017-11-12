/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.security.InvalidParameterException;
import java.util.Collection;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Geometry;

/**
 * This class provides several util functions for localization.
 *
 * @author Stefan Glaser
 *
 */
public class LocalizerUtil
{
	/**
	 * Calculates a normal vector to the plane defined by the point set. To do
	 * so, at least 3 points on the plane has to be passed.<br>
	 * This method performs an SVD of the covariance matrix to get the smallest
	 * eigenvector (which is already a normal vector of the least square fitting
	 * plane). After the calculation of the eigenvector it is ensured, that the
	 * returned normal vector always points towards the center of the coordinate
	 * system. (this means for example, if this method is used to compute the
	 * normal of the ground by a bunch of points on ground level, the
	 * resulting normal points always to the sky, since we can't be below the
	 * ground)
	 *
	 * @param points the points forming a plane
	 * @return a normal vector of the plane, always pointing towards the center
	 *         of the coordinate system
	 */
	public static Vector3D calculatePlaneNormal(List<Vector3D> points)
	{
		// Check for minimum 3 points
		if (points.size() < 3) {
			return null;
		}

		Vector3D centroid;
		double[][] cov = new double[points.size()][3];
		double x = 0, y = 0, z = 0;

		// Calculate centroid
		for (Vector3D vec : points) {
			x += vec.getX();
			y += vec.getY();
			z += vec.getZ();
		}
		centroid = new Vector3D(x / points.size(), y / points.size(), z / points.size());

		// Calculate co-variance matrix
		int i = 0;
		for (Vector3D vec : points) {
			cov[i][0] = vec.getX() - centroid.getX();
			cov[i][1] = vec.getY() - centroid.getY();
			cov[i][2] = vec.getZ() - centroid.getZ();
			i++;
		}

		// Compute SVD
		RealMatrix covMatrix = new Array2DRowRealMatrix(cov, false);
		SingularValueDecomposition svd = new SingularValueDecomposition(covMatrix);
		RealMatrix V = svd.getV();

		// Take the eigenvector to the smalles eigenvalue (which should in our
		// case be the third one - so take the third column of V)
		Vector3D res = new Vector3D(V.getEntry(0, 2), V.getEntry(1, 2), V.getEntry(2, 2));

		// Check if the normal points towards the bottom - and if so, let it point
		// towards the sky
		if (Vector3D.dotProduct(centroid.normalize(), res) > 0) {
			res = res.negate();
		}

		return res;
	}

	/**
	 * This method calculates the position from a set of reference points and the
	 * given orientation. The position is calculated by using the given
	 * orientation to transform the local position of a reference point into the
	 * global coordinate system. After this transformation, we can simply
	 * subtract this vector from the known position to retrieve our position.
	 * With more than one reference point, the resulting positions to the
	 * different reference points are simply averaged.
	 *
	 * @param refPoints - a collection of reference points for position
	 *        calculation
	 * @param orientation - the rotation to transform local positions into global
	 *        positions
	 * @return the position
	 */
	public static Vector3D calculatePosition(Collection<IReferencePoint> refPoints, Rotation orientation)
	{
		return calculatePosition(refPoints, null, orientation);
	}

	/**
	 * This method calculates the position from a set of reference points and
	 * reference lines and the given orientation. The position is calculated by
	 * using the given orientation to transform the local position of a reference
	 * point / reference line-point into the global coordinate system. After this
	 * transformation, we can simply subtract this vector from the known position
	 * to retrieve our position. With more than one reference point, the
	 * resulting positions to the different reference points are simply averaged.
	 *
	 * @param refPoints - a collection of reference points for position
	 *        calculation
	 * @param refLines - a collection of reference lines for position calculation
	 * @param orientation - the rotation to transform local positions into global
	 *        positions
	 * @return the position
	 */
	public static Vector3D calculatePosition(
			Collection<IReferencePoint> refPoints, Collection<IReferenceLine> refLines, Rotation orientation)
	{
		// localized x-, y-, z-Position
		double x = 0;
		double y = 0;
		double z = 0;
		int noOfUsedRefPoints = 0;

		Vector3D myPos;
		Vector3D localPosition;
		Vector3D knownPosition;

		// iterate through all reference points
		if (refPoints != null) {
			for (IReferencePoint flag : refPoints) {
				myPos = calculatePosition(flag.getLocalPosition(), flag.getKnownPosition(), orientation);

				// add calculated position
				x += myPos.getX();
				y += myPos.getY();
				z += myPos.getZ();
				noOfUsedRefPoints++;
			}
		}

		// iterate through all reference lines
		// TODO: Check if line is not seen completely and therefore not matching
		// both known positions and adjust the known position of the not matching
		// reference point
		if (refLines != null) {
			for (IReferenceLine line : refLines) {
				localPosition = line.getLocalPosition1();
				if (localPosition != null) {
					knownPosition = line.getAdjustedKnownPosition1();
					myPos = calculatePosition(localPosition, knownPosition, orientation);

					// add calculated position
					x += myPos.getX();
					y += myPos.getY();
					z += myPos.getZ();
					noOfUsedRefPoints++;
				}

				localPosition = line.getLocalPosition1();
				if (localPosition != null) {
					knownPosition = line.getAdjustedKnownPosition2();
					myPos = calculatePosition(localPosition, knownPosition, orientation);

					// add calculated position
					x += myPos.getX();
					y += myPos.getY();
					z += myPos.getZ();
					noOfUsedRefPoints++;
				}
			}
		}

		if (noOfUsedRefPoints == 0) {
			return null;
		}

		// average results
		x /= noOfUsedRefPoints;
		y /= noOfUsedRefPoints;
		z /= noOfUsedRefPoints;

		return new Vector3D(x, y, z);
	}

	/**
	 * This method calculates the position from a set of reference points. We can
	 * simply subtract this vector from the known position to retrieve our
	 * position. because the referenced points are already transformed into to
	 * global system. With more than one reference point, the resulting positions
	 * to the different reference points are simply averaged.
	 *
	 * @param refPoints - a collection of global reference points for position
	 *        calculation
	 * @return the position
	 */
	public static Vector3D calculatePositionByGlobalPoints(Collection<IReferencePoint> refPoints)
	{
		// localized x-, y-, z-Position
		double x = 0;
		double y = 0;
		double z = 0;
		int noOfUsedRefPoints = 0;

		Vector3D myPos;

		// iterate through all reference points
		if (refPoints != null) {
			for (IReferencePoint flag : refPoints) {
				myPos = flag.getKnownPosition().subtract(flag.getLocalPosition());

				// add calculated position
				x += myPos.getX();
				y += myPos.getY();
				z += myPos.getZ();
				noOfUsedRefPoints++;
			}
		}

		if (noOfUsedRefPoints == 0) {
			return null;
		}

		// average results
		x /= noOfUsedRefPoints;
		y /= noOfUsedRefPoints;
		z /= noOfUsedRefPoints;

		return new Vector3D(x, y, z);
	}

	/**
	 * This method calculates the position based on a single local-to-known
	 * position combination and the given orientation.
	 *
	 * @param localPosition - the position in the global body system
	 * @param knownPosition - the known position in the global system
	 * @param orientation - the transformation from the local to the global
	 *        system
	 * @return resulting position
	 */
	private static Vector3D calculatePosition(Vector3D localPosition, Vector3D knownPosition, Rotation orientation)
	{
		return knownPosition.subtract(orientation.applyTo(localPosition));
	}

	/**
	 * An implementation of the umeyama point set registration method (known from
	 * image processing). The result is an 3D rotation, which aligns the given
	 * moving point set best to the fixed point set, in the sense of least square
	 * distances.
	 *
	 * @param movingVec - the array of moving points to register
	 * @param fixedVec - the array of corresponding fixed points
	 * @return the 3D rotation, which aligns the given moving point set best to
	 *         the fixed point set, in the sense of least square distances
	 */
	public static Rotation umeyama(Vector3D[] movingVec, Vector3D[] fixedVec)
	{
		if (movingVec.length != fixedVec.length) {
			throw new InvalidParameterException();
		}

		double[][] cov = new double[][] {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

		for (int i = 0; i < movingVec.length; i++) {
			cov[0][0] += fixedVec[i].getX() * movingVec[i].getX();
			cov[0][1] += fixedVec[i].getX() * movingVec[i].getY();
			cov[0][2] += fixedVec[i].getX() * movingVec[i].getZ();

			cov[1][0] += fixedVec[i].getY() * movingVec[i].getX();
			cov[1][1] += fixedVec[i].getY() * movingVec[i].getY();
			cov[1][2] += fixedVec[i].getY() * movingVec[i].getZ();

			cov[2][0] += fixedVec[i].getZ() * movingVec[i].getX();
			cov[2][1] += fixedVec[i].getZ() * movingVec[i].getY();
			cov[2][2] += fixedVec[i].getZ() * movingVec[i].getZ();
		}

		// Compute SVD
		RealMatrix covMatrix = new Array2DRowRealMatrix(cov, false);
		SingularValueDecomposition svd = new SingularValueDecomposition(covMatrix);

		RealMatrix U = svd.getU();
		RealMatrix S = new Array2DRowRealMatrix(new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
		RealMatrix V = svd.getV();

		LUDecomposition U_LUDec = new LUDecomposition(U);
		LUDecomposition V_LUDec = new LUDecomposition(U);

		if (U_LUDec.getDeterminant() * V_LUDec.getDeterminant() == -1) {
			S.setEntry(2, 2, -1);
		}
		U = U.multiply(S).multiply(V.transpose());

		return Geometry.toRotation(U);
	}
}
