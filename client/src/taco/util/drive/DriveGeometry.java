package taco.util.drive;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import hso.autonomy.util.misc.FuzzyCompare;

public class DriveGeometry
{
	private static final double THRESHOLD = 5;

	private static final double MINIMUM_RADIUS = 0.5;

	private static final double CAR_POSE_ADVANCE = 0.15;

	private static final double BACKWARDS_THRESHOLD = -0.55;

	/** the distance between the front and rear axle */
	private double axleSpacing;

	private Angle minAngle;

	private Angle maxAngle;

	private SteerInstruction previousInstruction;

	public DriveGeometry(double axleSpacing, Angle minAngle, Angle maxAngle)
	{
		this.axleSpacing = axleSpacing;
		this.minAngle = minAngle;
		this.maxAngle = maxAngle;
		previousInstruction = new SteerInstruction();
	}

	/**
	 * Calculates the 2D position and orientation on a circular arc with the passed length and current rotation
	 * @param length the length of the arc segment
	 * @param exitAngle the angle of the end of the arc relative to the start
	 * @return the pose of the end of the arc relative to the start
	 */
	public static Pose2D calculateArcPose(double length, Angle exitAngle)
	{
		double x = length;
		double y = 0;
		double angle = exitAngle.radians();
		if (!FuzzyCompare.isZero(angle)) {
			double radius = length / angle;
			x = Math.sin(angle) * radius;
			y = radius * (1 - Math.cos(angle));
		}

		return new Pose2D(x, y, exitAngle);
	}

	public void resetPreviousInstruction()
	{
		previousInstruction = new SteerInstruction();
	}

	public void resetPreviousInstruction(Angle angle, boolean driveForwards)
	{
		previousInstruction = new SteerInstruction(angle, driveForwards);
	}

	/**
	 * Returns a SteerInstruction-Object, which contains information about the direction that should be driven,
	 * as well as the angle.
	 * @param forceBackwards
	 */
	public SteerInstruction getNextInstruction(IPose2D currentPose, IPose2D nextWaypoint, IPose2D waypointAfterNext,
			boolean allowBackwards, boolean forceBackwards)
	{
		SteerInstruction steerInstruction =
				getSteerInstruction(currentPose, nextWaypoint, waypointAfterNext, allowBackwards, forceBackwards);
		if (steerInstruction.steeringAngle == null) {
			steerInstruction = previousInstruction;
		}

		if (steerInstruction.driveForward != previousInstruction.driveForward) {
			double distanceDotProduct = getPoseDotProduct(currentPose, nextWaypoint);

			// Prevent "flickering" of forwards and backwards-driving
			if ((distanceDotProduct < DriveGeometry.BACKWARDS_THRESHOLD + 0.2 &&
						distanceDotProduct > DriveGeometry.BACKWARDS_THRESHOLD - 0.2) ||
					currentPose.getDistanceTo(nextWaypoint) < 0.1) {
				steerInstruction = previousInstruction;
			}
		}
		previousInstruction = steerInstruction;
		return steerInstruction;
	}

	SteerInstruction getSteerInstruction(IPose2D currentPose, IPose2D nextWaypoint, IPose2D waypointAfterNext,
			boolean allowBackwards, boolean forceBackwards)
	{
		if (currentPose.getDistanceTo(nextWaypoint) < axleSpacing && waypointAfterNext != null) {
			return getSteerInstruction(currentPose, waypointAfterNext, null, allowBackwards, forceBackwards);
		}
		if (!allowBackwards) {
			return new SteerInstruction(
					getSteeringAngle(currentPose, nextWaypoint, axleSpacing, minAngle, maxAngle), true);
		} else if (forceBackwards) {
			return new SteerInstruction(
					getBackwardsSteeringAngle(currentPose, nextWaypoint, minAngle, maxAngle), false);
		}
		boolean driveBackward = shouldDriveBackwards(currentPose, nextWaypoint);
		Angle angle;
		if (!driveBackward) {
			angle = getSteeringAngle(currentPose, nextWaypoint, axleSpacing, minAngle, maxAngle);
		} else {
			angle = getBackwardsSteeringAngle(currentPose, nextWaypoint, minAngle, maxAngle);
		}

		return new SteerInstruction(angle, !driveBackward);
	}

	/**
	 * Calculate the driving pose based on the rear wheel spacing and their rolling-distances.
	 *
	 * Calculate the pose of the (center of the) rear axle after rolling leftWheelDistance and rightWheelDistance on the
	 * wheels. In order to to so, the wheel spacing (distance between the wheels along the axle).
	 *
	 * @param leftWheelDistance - the distance the left wheel progressed
	 * @param rightWheelDistance - the distance the right wheel progressed
	 * @param wheelSpacing - the distance between the two wheels along the axle
	 */
	static Pose2D calculateCurvePose(double leftWheelDistance, double rightWheelDistance, double wheelSpacing)
	{
		double x, y, alpha;

		if (leftWheelDistance == rightWheelDistance) {
			// Both wheels spin in the same direction with the same speed, thus we are driving in a straight line.
			x = rightWheelDistance;
			y = 0;
			alpha = 0;

		} else if (leftWheelDistance == -rightWheelDistance) {
			// Both wheels spin at the same speed, but in different directions, thus we are turing on spot.
			x = 0;
			y = 0;
			alpha = 2 * rightWheelDistance / wheelSpacing;

		} else {
			double innerWheelDistance = leftWheelDistance;
			double outerWheelDistance = rightWheelDistance;
			int sideFactor = 1;

			if (Math.abs(leftWheelDistance) > Math.abs(rightWheelDistance)) {
				innerWheelDistance = rightWheelDistance;
				outerWheelDistance = leftWheelDistance;
				sideFactor = -1;
			}

			if (leftWheelDistance * rightWheelDistance >= 0) {
				// Both wheels spin in the same direction, thus the origin of the driving circle is outside (left or
				// right) the car.

				double diffDistance = outerWheelDistance - innerWheelDistance;
				double r = (wheelSpacing * innerWheelDistance / diffDistance) + (wheelSpacing / 2);

				alpha = sideFactor * (diffDistance / wheelSpacing);
				x = sideFactor * (Math.sin(alpha) * r);
				y = -sideFactor * (Math.cos(alpha) * r - r);

			} else {
				// The wheels spin in different directions, thus the origin of the driving circle is between the two
				// wheels.

				double distanceSum = innerWheelDistance + outerWheelDistance;
				double r = (wheelSpacing / 2) - (wheelSpacing * innerWheelDistance / distanceSum);

				alpha = sideFactor * (distanceSum / wheelSpacing);
				x = sideFactor * (Math.sin(alpha) * r);
				y = -sideFactor * (Math.cos(alpha) * r - r);
			}
		}

		return new Pose2D(x, y, Angle.rad(alpha));
	}

	/**
	 * Calculate the curve radius from a certain steering angle.
	 *
	 * Calculate the resulting curve radius by driving with the given steering angle.
	 * Note: A positive/negative steering angle results in an positive/negative curve radius.
	 *
	 * @param steeringAngle - the steering angle
	 * @return the curve radius resulting from the given steering angle
	 */
	public double calculateCurveRadius(Angle steeringAngle)
	{
		double steering = steeringAngle.radians();
		if (Math.abs(steering) >= Math.PI / 2) {
			return 0;
		} else if (steering == 0) {
			return Double.MAX_VALUE;
		}

		return axleSpacing / Math.tan(steering);
	}

	/**
	 * Calculate the steering angle to drive a certain curve.
	 *
	 * Calculate the necessary steering angle to drive a curve with the given radius.
	 * Note: A positive/negative curve radius results in an positive/negative steering angle.
	 *
	 * @param curveRadius - the radius of the intended curve in meter
	 * @return the steering angle resulting in the given curve radius
	 */
	Angle calculateSteeringAngle(double curveRadius)
	{
		if (curveRadius < 0.000001 && curveRadius >= 0.0) {
			return Angle.ANGLE_90;
		} else if (curveRadius > -0.000001 && curveRadius <= 0.0) {
			return Angle.ANGLE_90.negate();
		}

		return Angle.rad(Math.atan(Math.abs(axleSpacing) / curveRadius));
	}

	public Circle2D getCircle(IPose2D currentPose, IPose2D nextWaypoint, IPose2D backAxlePose, boolean allowBackwards)
	{
		Circle2D circle = new Circle2D(nextWaypoint, currentPose.getPosition());

		if (Double.isNaN(circle.getRadius())) {
			return null;
		} else if (circle.getDirectionOfRotation(nextWaypoint) != circle.getDirectionOfRotation(currentPose)) {
			if (backAxlePose != null) {
				return getCircle(backAxlePose, nextWaypoint, null, allowBackwards);
			}
			if (!allowBackwards) {
				return null;
			}
		} else if (circle.getRadius() < MINIMUM_RADIUS) {
			//			return null;
		}
		return circle;
	}

	public Line2D getLine(IPose2D currentPose, IPose2D nextWaypoint, boolean allowBackwards)
	{
		Line2D line = new Line2D(currentPose.getPosition(), nextWaypoint.getPosition());
		if (!allowBackwards && Math.abs(line.getAngle().subtract(currentPose.getAngle()).degrees()) > THRESHOLD) {
			return null;
		} else if (allowBackwards &&
				   Math.abs(line.getAngle().subtract(nextWaypoint.getAngle()).degrees()) > THRESHOLD) {
			return null;
		}
		return line;
	}

	IBehaviorGeometry chooseDriveGeometry(
			IPose2D currentPose, IPose2D nextWaypoint, IPose2D backAxlePose, boolean allowBackwards)
	{
		IBehaviorGeometry behaviorGeometry = getLine(currentPose, nextWaypoint, allowBackwards);
		if (behaviorGeometry != null) {
			return behaviorGeometry;
		}
		behaviorGeometry = getCircle(currentPose, nextWaypoint, backAxlePose, allowBackwards);
		if (behaviorGeometry != null) {
			return behaviorGeometry;
		}
		return null;
	}

	/**
	 * Returns an angle that can be used to reach a pose from another.
	 * @param currentPose - an initial, current Pose
	 * @param nextWaypoint - the pose that shall be reached starting from currentPose
	 * @return Angle An Angle that can be driven in order to get closer to nextWaypoint
	 */
	private Angle getSteeringAngle(
			IPose2D currentPose, IPose2D nextWaypoint, double axleSpacing, Angle minAngle, Angle maxAngle)
	{
		IPose2D backAxlePose = currentPose;
		currentPose = currentPose.applyTo(new Pose2D(axleSpacing, 0));

		IBehaviorGeometry geometry = chooseDriveGeometry(currentPose, nextWaypoint, backAxlePose, false);
		if (geometry == null) {
			return null;
		}
		Vector3D virtualPoint = currentPose.getPosition().add(
				AngleUtils.get3DVectorFromAngle(currentPose.getAngle().radians(), CAR_POSE_ADVANCE));

		IPose2D globalVirtualPose = geometry.getClosestPose(virtualPoint);

		IPose2D target = currentPose.applyInverseTo(globalVirtualPose);

		Angle angle = AngleUtils.to(target.getX(), target.getY());
		return AngleUtils.limit(angle, minAngle, maxAngle);
	}

	private Angle getBackwardsSteeringAngle(IPose2D currentPose, IPose2D backwardsPoint, Angle minAngle, Angle maxAngle)
	{
		IBehaviorGeometry geometry = chooseDriveGeometry(backwardsPoint, currentPose, null, true);
		if (geometry == null) {
			return null;
		}
		Angle backwardsAngle = currentPose.getAngle();

		if (geometry.getClass().getName().equals(Line2D.class.getName())) {
			backwardsAngle = ((Line2D) geometry).getAngle();
		}
		Vector3D virtualPoint =
				currentPose.getPosition().subtract(AngleUtils.get3DVectorFromAngle(backwardsAngle.radians(), 0.15));
		IPose2D globalVirtualPose;

		globalVirtualPose = geometry.getClosestPose(virtualPoint);
		IPose2D target;
		// This distinction is important for accuracy:
		if (geometry.getClass().getName().equals(Circle2D.class.getName())) {
			target = currentPose.applyInverseTo(globalVirtualPose);
		} else {
			target = globalVirtualPose.applyInverseTo(currentPose);
		}

		Angle angle = AngleUtils.to(target.getX(), target.getY());
		return AngleUtils.limit(angle, minAngle, maxAngle);
	}

	double getPoseDotProduct(IPose2D currentPose, IPose2D nextPose)
	{
		Line2D frontAxleLine = new Line2D(currentPose.getX(), currentPose.getY(), currentPose.getAngle(), axleSpacing);

		Vector3D subtract = new Vector3D(frontAxleLine.getEnd().getX(), frontAxleLine.getEnd().getY(), 0)
									.subtract(currentPose.getPosition());
		if (subtract.getNorm() < 0.0001) {
			return 1;
		}
		Vector3D carVector = subtract.normalize();

		Vector3D waypointVector = nextPose.getPosition().subtract(currentPose.getPosition());
		if (waypointVector.getNorm() < 0.0001) {
			return 1;
		}
		waypointVector = waypointVector.normalize();
		return waypointVector.dotProduct(carVector);
	}

	public static double getPoseDotProduct(IPose2D currentPose, IPose2D nextPose, double axleSpacing)
	{
		Line2D frontAxleLine = new Line2D(currentPose.getX(), currentPose.getY(), currentPose.getAngle(), axleSpacing);

		Vector3D subtract = new Vector3D(frontAxleLine.getEnd().getX(), frontAxleLine.getEnd().getY(), 0)
									.subtract(currentPose.getPosition());
		if (subtract.getNorm() < 0.0001) {
			return 1;
		}
		Vector3D carVector = subtract.normalize();

		Vector3D waypointVector = nextPose.getPosition().subtract(currentPose.getPosition());
		if (waypointVector.getNorm() < 0.0001) {
			return 1;
		}
		waypointVector = waypointVector.normalize();
		return waypointVector.dotProduct(carVector);
	}

	private boolean shouldDriveBackwards(IPose2D currentPose, IPose2D nextPose)
	{
		return getPoseDotProduct(currentPose, nextPose) < BACKWARDS_THRESHOLD;
	}

	//	public IBehaviorGeometry getSCurveGeometry(
	//			IPose2D currentPose, IPose2D nextWaypoint, IPose2D startPoint, SegmentType currentSegmentType)
	//	{
	//		if (currentSegmentType == SegmentType.S_CURVE_BOTTOM ||
	//				currentSegmentType ==
	//						SegmentType.S_CURVE_TOP) { // We most likely have a S Curve - as long as we drive via
	//												   // map with them marked. It'd be better to implement multiple
	//												   // conditions though, since this isn't sustainable.
	//			return new SCurve(startPoint, nextWaypoint);
	//		}
	//		return null;
	//	}
}