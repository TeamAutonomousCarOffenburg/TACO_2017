/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.CardanEulerSingularityException;
import org.apache.commons.math3.geometry.euclidean.threed.NotARotationMatrixException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * Utilities for geometric calculations
 *
 * @author Klaus Dorer
 */
public class Geometry
{
	/**
	 * Returns a linear interpolation between the two passed points at the passed
	 * x coordinate. Depending on the ascending parameter the function grows
	 * (true) from 0 to 1 otherwise it decreases from 1 to 0. Outside the range
	 * specified by x0 and x1 the values are 0 (left, true or right, false) or 1
	 * respectively (left, false or right, true.
	 * @param x0 lower x value
	 * @param x1 higher x value
	 * @param ascending true if the lower x value represents 0 false if it
	 *        represents 1
	 * @param x the x value for which to get the fuzzy value
	 * @return a fuzzy value that is linearly interpolated between the two passed
	 *         values
	 */
	public static double getLinearFuzzyValue(double x0, double x1, boolean ascending, double x)
	{
		if (x0 > x1) {
			double temp = x0;
			x0 = x1;
			x1 = temp;
		}

		if (ascending) {
			if (x <= x0) {
				return 0.0;
			}
			if (x >= x1) {
				return 1.0;
			}
			return linearInterpolation(x0, 0, x1, 1, x);
		}

		if (x <= x0) {
			return 1.0;
		}

		if (x >= x1) {
			return 0.0;
		}

		return linearInterpolation(x0, 1, x1, 0, x);
	}

	/**
	 * Returns the linear interpolation (y value) of the passed points (x0,y0)
	 * and (x1,y1) at the passed coordinate x
	 * @param x0 x coordinate of the first point
	 * @param y0 y coordinate of the first point
	 * @param x1 x coordinate of the second point
	 * @param y1 y coordinate of the second point
	 * @param x x coordinate of the point of interest
	 * @return the y coordinate of position x as linear interpolation of the two
	 *         passed points
	 */
	public static double linearInterpolation(double x0, double y0, double x1, double y1, double x)
	{
		return y0 + (y1 - y0) / (x1 - x0) * (x - x0);
	}

	/**
	 * @param source source point
	 * @param destination destination point
	 * @param distanceFromSource distance from source point
	 * @return a point on the line between source and destination with distance
	 *         from source as specified
	 */
	public static Pose2D getPointInterpolation(Vector3D source, Vector3D destination, double distanceFromSource)
	{
		Vector3D srcDest = destination.subtract(source);
		if (srcDest.getNorm() < 0.001) {
			// to avoid normalization problems
			return new Pose2D(source);
		}
		srcDest = srcDest.normalize().scalarMultiply(distanceFromSource);
		Angle directionSrcDest = Angle.rad(srcDest.getAlpha());
		srcDest = srcDest.add(source);
		return new Pose2D(srcDest, directionSrcDest);
	}

	/**
	 * Create a Rotation representing a rotation around the x-axis.
	 *
	 * @param angle - the turn angle (rad)
	 * @return a rotation around the x-axis about angle
	 */
	public static Rotation createXRotation(double angle)
	{
		return new Rotation(Vector3D.PLUS_I, angle);
	}

	/**
	 * Create a Rotation representing a rotation around the y-axis.
	 *
	 * @param angle - the turn angle (rad)
	 * @return a rotation around the y-axis about angle
	 */
	public static Rotation createYRotation(double angle)
	{
		return new Rotation(Vector3D.PLUS_J, angle);
	}

	/**
	 * Create a Rotation representing a rotation around the z-axis.
	 *
	 * @param angle - the turn angle (rad)
	 * @return a rotation around the z-axis about angle
	 */
	public static Rotation createZRotation(double angle)
	{
		return new Rotation(Vector3D.PLUS_K, angle);
	}

	/**
	 * Extract the rotation about the z-axis from a given rotation.
	 *
	 * @param rotation the rotation
	 * @return the z-axis rotation angle
	 */
	public static Angle getHorizontalAngle(Rotation rotation)
	{
		try {
			double[] angles = rotation.getAngles(RotationOrder.ZXY);
			return Angle.rad(angles[0]);
		} catch (CardanEulerSingularityException e) {
			e.printStackTrace();
		}

		return Angle.ZERO;
	}

	/**
	 * Transform a given matrix to a rotation.
	 *
	 * @param matrix the matrix containing a rotation
	 * @return a Rotation representing the same rotation as the given matrix
	 */
	public static Rotation toRotation(RealMatrix matrix)
	{
		try {
			return new Rotation(matrix.getData(), 1.0e-8);
		} catch (NotARotationMatrixException e) {
			e.printStackTrace();
		}

		return Rotation.IDENTITY;
	}

	/**
	 * Transforms the given rotation relative to a system that is rotated around
	 * the x-axis by xAngle.
	 *
	 * @param rotation - the rotation to transform
	 * @param xAngle - difference x-angle in rad
	 *
	 * @return the given rotation relative to a system that is rotated by xAngle
	 *         around the x-axis
	 */
	public static Rotation xTransformRotation(Rotation rotation, double xAngle)
	{
		Rotation diffRot = createXRotation(xAngle);
		return diffRot.applyTo(rotation).applyTo(diffRot.revert());
	}

	/**
	 * Transforms the given rotation relative to a system that is rotated around
	 * the y-axis by yAngle.
	 *
	 * @param rotation - the rotation to transform
	 * @param yAngle - difference y-angle in rad
	 *
	 * @return the given rotation relative to a system that is rotated by yAngle
	 *         around the y-axis
	 */
	public static Rotation yTransformRotation(Rotation rotation, double yAngle)
	{
		Rotation diffRot = createYRotation(yAngle);
		return diffRot.applyTo(rotation).applyTo(diffRot.revert());
	}

	/**
	 * Transforms the given rotation relative to a system that is rotated around
	 * the z-axis by zAngle.
	 *
	 * @param rotation - the rotation to transform
	 * @param zAngle - difference z-angle in rad
	 *
	 * @return the given rotation relative to a system that is rotated by zAngle
	 *         around the z-axis
	 */
	public static Rotation zTransformRotation(Rotation rotation, double zAngle)
	{
		Rotation diffRot = createZRotation(zAngle);
		return diffRot.applyTo(rotation).applyTo(diffRot.revert());
	}

	public static Pose3D bodyToWorld(IPose3D bodyPose)
	{
		Vector3D position = new Vector3D(bodyPose.getY(), -bodyPose.getX(), bodyPose.getZ());
		Rotation rotation = Geometry.zTransformRotation(bodyPose.getOrientation(), -Math.PI / 2);
		return new Pose3D(position, rotation);
	}

	/**
	 * Processes the top view z-normalized orientation.
	 *
	 * @param orientation - global orientation
	 * @return the given orientation without z-rotation
	 */
	public static Rotation getTopViewOrientation(Rotation orientation)
	{
		// Process the top view z-normalized orientation
		double angle = getTopViewZAngle(orientation);
		return new Rotation(Vector3D.PLUS_K, angle).applyTo(orientation);
	}

	/**
	 * Processes the top view z-rotation.
	 *
	 * @param orientation - global orientation
	 * @return the given top view z-rotation in rad
	 */
	public static double getTopViewZAngle(Rotation orientation)
	{
		// Process the top view z-normalized orientation
		double[][] tempMat = orientation.getMatrix();
		return Math.atan2(tempMat[0][1], tempMat[1][1]);
	}

	/**
	 * Print a rotation matrix.
	 * @param rot - the rotation to print.
	 */
	public static void printRotationMatrix(Rotation rot)
	{
		printMatrix(rot.getMatrix());
	}

	/**
	 * Print a matrix.
	 * @param matrix the matrix to print.
	 */
	public static void printMatrix(double[][] matrix)
	{
		for (int row = 0; row < matrix.length; row++) {
			for (int col = 0; col < matrix[0].length; col++) {
				if (col > 0) {
					System.out.print(" ");
				}
				System.out.print(matrix[row][col]);
			}
			System.out.println();
		}
	}

	/**
	 * Calculates a point on the line from start to end that is distance away
	 * from the end point towards the start point.
	 * @param start the start point of the line
	 * @param end the end point of the line
	 * @param distance the distance from end (absolute)
	 * @return the point
	 */
	public static Vector3D getPointOnLineAbsoluteEnd(Vector3D start, Vector3D end, final double distance)
	{
		Vector3D endStart = start.subtract(end);
		if (endStart.getNorm() < 0.001) {
			// avoid vector normalization exception
			return end;
		}
		return end.add(endStart.normalize().scalarMultiply(distance));
	}

	/**
	 * Calculates a point that is orthogonal to the start - end line (in 2D) at
	 * the point that is distanceEnd away from end and distanceLeft away from
	 * start - end
	 * @param start the start point of the line
	 * @param end the end point
	 * @param distanceEnd the distance from end (absolute) towards start
	 * @param distanceLeft the distance of the point from the line
	 * @return the point as calculated above
	 */
	public static Vector3D getPointOnOrthogonal2D(Vector3D start, Vector3D end, double distanceEnd, double distanceLeft)
	{
		Vector3D start2D = new Vector3D(start.getX(), start.getY(), 0);
		Vector3D end2D = new Vector3D(end.getX(), end.getY(), 0);
		Vector3D onLine2D = getPointOnLineAbsoluteEnd(start2D, end2D, distanceEnd);
		Vector3D endStart2D = start2D.subtract(end2D);
		if (endStart2D.getNorm() < 0.001) {
			// avoid vector normalization exception
			return end2D;
		}
		Vector3D endStartNorm = endStart2D.normalize();
		Vector3D orthogonal2D = new Vector3D(endStartNorm.getY(), -endStartNorm.getX(), 0);
		return onLine2D.add(orthogonal2D.scalarMultiply(distanceLeft));
	}

	/**
	 * Returns an array of future positions as predicted by the speed.
	 *
	 * @param position The current position of the object.
	 * @param speedVector The speed vector of the object.
	 * @param howMany The number of future positions that should be returned
	 * @return an array of future positions
	 */
	public static Vector3D[] getFuturePositions(Vector3D position, Vector3D speedVector, int howMany)
	{
		Vector3D[] futurePositions = new Vector3D[howMany];

		futurePositions[0] = position.add(speedVector);
		for (int i = 1; i < howMany; i++) {
			futurePositions[i] = futurePositions[i - 1].add(speedVector);
		}
		return futurePositions;
	}

	/**
	 * Returns an array of future positions as predicted by the speed under the
	 * influence of gravitational acceleration and air resistance.
	 *
	 * @param position The current position of the Ball.
	 * @param speedVector The speed vector of the Ball.
	 * @param ballDecay The ball decay.
	 * @param radius The radius of the Ball.
	 * @param howMany The number of future positions that should be returned.
	 * @param cycleTime The simulations cycle time.
	 * @return an array of future positions
	 */
	public static Vector3D[] getFuturePositions(
			Vector3D position, Vector3D speedVector, float ballDecay, float radius, int howMany, float cycleTime)
	{
		final double drag = -0.4 * cycleTime;
		final double dragZ = 9.81 * cycleTime;
		final double bounceXY = 0.75;
		final double bounceZ = -0.67;

		Vector3D[] futurePositions = new Vector3D[howMany];
		Vector3D currentPosition = position;
		Vector3D velocity = speedVector;

		boolean flying = currentPosition.getZ() > radius;
		int numBounces = 0;

		for (int i = 0; i < howMany; i++) {
			Vector3D oldVelocity = velocity;

			if (flying) {
				velocity = velocity.add(new Vector3D(
						velocity.getX() * drag, velocity.getY() * drag, -dragZ * cycleTime + velocity.getZ() * drag));
			} else {
				velocity = velocity.scalarMultiply(ballDecay);
			}

			currentPosition = currentPosition.add(oldVelocity.add(velocity).scalarMultiply(0.5));
			if (currentPosition.getZ() < radius) {
				currentPosition = new Vector3D(currentPosition.getX(), currentPosition.getY(), radius);
				if (numBounces < 3) {
					numBounces++;
					velocity = new Vector3D(
							velocity.getX() * bounceXY, velocity.getY() * bounceXY, velocity.getZ() * bounceZ);
					flying = true;
				} else {
					velocity = new Vector3D(velocity.getX(), velocity.getY(), 0);
					flying = false;
				}
			}
			futurePositions[i] = currentPosition;
		}
		return futurePositions;
	}

	public static Vector3D getInterceptionPoint(Vector3D startPosition, Vector3D[] targetPositions, double cycleSpeed)
	{
		int left = 0;
		int right = targetPositions.length - 1;
		while (left < right) {
			int i = (left + right) / 2;
			double distance = startPosition.subtract(targetPositions[i]).getNorm();
			double possibleDistance = cycleSpeed * i;
			if (possibleDistance < distance) {
				left = i + 1;
			} else {
				right = i;
			}
		}
		return targetPositions[right];
	}

	public static Rotation getAverageRotation(List<Rotation> rotations)
	{
		if (rotations == null || rotations.size() == 0) {
			return Rotation.IDENTITY;
		}

		Rotation currentAvgRot = rotations.get(0);

		// Temporary values
		double w = currentAvgRot.getQ0();
		double x = currentAvgRot.getQ1();
		double y = currentAvgRot.getQ2();
		double z = currentAvgRot.getQ3();
		double factor;
		Rotation currentRot;

		// Loop through all the rotational values.
		for (int i = 1; i < rotations.size(); i++) {
			currentRot = rotations.get(i);

			double dot = currentAvgRot.getQ0() * currentRot.getQ0() + currentAvgRot.getQ1() * currentRot.getQ1() +
						 currentAvgRot.getQ2() * currentRot.getQ2() + currentAvgRot.getQ3() * currentRot.getQ3();

			if (dot < 0) {
				w -= currentRot.getQ0();
				x -= currentRot.getQ1();
				y -= currentRot.getQ2();
				z -= currentRot.getQ3();
			} else {
				w += currentRot.getQ0();
				x += currentRot.getQ1();
				y += currentRot.getQ2();
				z += currentRot.getQ3();
			}

			factor = 1.0 / i;
			currentAvgRot = new Rotation(w * factor, x * factor, y * factor, z * factor, true);
		}

		return currentAvgRot;
	}
}
