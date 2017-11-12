/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

public class Circle2D
{
	/** x coordinate of the circle */
	private double x;

	/** y coordinate of the circle */
	private double y;

	/** radius of the circle */
	private double radius;

	public Circle2D()
	{
		this(0, 0, 0);
	}

	public Circle2D(double x, double y)
	{
		this(x, y, 0);
	}

	public Circle2D(Circle2D other)
	{
		this(other.x, other.y, other.radius);
	}

	public Circle2D(double x, double y, double radius)
	{
		this.x = x;
		this.y = y;
		this.radius = radius;
	}

	public double getX()
	{
		return x;
	}

	public void setX(double x)
	{
		this.x = x;
	}

	public double getY()
	{
		return y;
	}

	public void setY(double y)
	{
		this.y = y;
	}

	public double getRadius()
	{
		return radius;
	}

	public void setRadius(double radius)
	{
		this.radius = radius;
	}

	// ### Methods ####
	/**
	 * calculates a point on the circle by given angle
	 * @return point on circle
	 */
	public Pose2D getPointOnCircle(Angle angle)
	{
		double x = Math.cos(angle.radians()) * this.radius;
		double y = Math.sin(angle.radians()) * this.radius;
		return new Pose2D(this.x + x, this.y + y);
	}

	/**
	 * checks if a point is on the circle line and calculates the angle of it (0
	 * to 360 degrees)
	 * @return return angle of point, or null if its not on circle
	 */
	public Angle getAngleToPoint(Pose2D point)
	{
		return new Pose2D(x, y).getAngleTo(new Pose2D(point.x, point.y));
	}

	/**
	 * checks if a point is located on a circle
	 * @return true if the point is on the circle, else false
	 */
	public boolean checkPointOnCircleArea(Pose2D point)
	{
		return this.getDistance(point) <= this.radius;
	}

	/**
	 * checks if two circles are touching in one point
	 * @return true if touching, else false
	 */
	public boolean checkOuterTouch(Circle2D other)
	{
		double dist = new Pose2D(x, y).getDistanceTo(new Pose2D(other.x, other.y));
		return dist == (this.getRadius() + other.getRadius());
	}

	/**
	 * checks if the circle other is placed in the current or current in the
	 * other one and they touching each other in one point
	 * @return true if touching, else false
	 */
	public boolean checkInnerTouch(Circle2D other)
	{
		double dist = new Pose2D(x, y).getDistanceTo(new Pose2D(other.x, other.y));
		return dist > 0 && (dist + other.radius == this.radius || dist + this.radius == other.radius);
	}

	/**
	 * checks if 2 circles intersect each other -> they got 2 cut points
	 * @return true if intersect, else false
	 */
	public boolean checkIntersect(Circle2D other)
	{
		double dist = new Pose2D(x, y).getDistanceTo(new Pose2D(other.x, other.y));
		if (dist != 0 && dist < (this.getRadius() + other.getRadius())) {
			if (!this.checkInnerTouch(other)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * one circle is placed in another one, they don't have any cut point
	 * @return true if one circle is in the other one, else false
	 */
	public boolean checkInnerCircle(Circle2D other)
	{
		double dist = this.getDistance(other);
		return dist + other.radius < this.radius || dist + this.radius < other.radius;
	}

	/**
	 * places a circle to a given pose
	 * @param pose - new location(Pose2D(x,y)) of circle
	 */
	public void relocate(Pose2D pose)
	{
		relocate(pose.x, pose.y);
	}

	/**
	 * places a circle to a given location(x/y)
	 * @param x - x coordinate of new location
	 * @param y - y coordinate of new location
	 */
	public void relocate(double x, double y)
	{
		this.x = x;
		this.y = y;
	}

	/**
	 * @return the distance
	 */
	public double getDistance(Pose2D other)
	{
		Pose2D selfPose = new Pose2D(x, y);
		return selfPose.getDistanceTo(other);
	}

	/**
	 * calculates distance to another Circle2D
	 * @return the distance
	 */
	public double getDistance(Circle2D other)
	{
		Pose2D otherPose = new Pose2D(other.x, other.y);
		return getDistance(otherPose);
	}

	@Override
	public boolean equals(Object other)
	{
		if (!(other instanceof Circle2D))
			return false;
		Circle2D c = (Circle2D) other;
		return (this.x == c.getX() && this.y == c.getY() && this.radius == c.getRadius());
	}

	/**
	 * calculates tangent by given another circle and the tangent number
	 * @param other circle fo calculation
	 * @param TangNo that is needed
	 * @return the calculated tangent
	 */
	public Tangent calculateTangent(Circle2D other, int TangNo)
	{
		Tangent tangent;
		if (TangNo % 2 == 1) {
			Circle2D curr = this;
			boolean swapped = false;

			if (curr.radius < other.radius) {
				Circle2D dummy = curr;
				curr = other;
				other = dummy;
				TangNo = (TangNo + 2) % 4;
				swapped = true;
			}

			double dist = curr.getDistance(other);
			double circDiff = curr.radius - other.radius;
			Angle beta = Angle.rad(Math.acos(circDiff / dist));
			Pose2D test = new Pose2D(curr.x, curr.y);
			Angle alpha = test.getAngleTo(new Pose2D(other.x, other.y));

			if (TangNo == 1) {
				beta = beta.negate();
			}
			Pose2D poseOnC1 = curr.getPointOnCircle(alpha.add(beta));
			Pose2D poseOnC2 = other.getPointOnCircle(alpha.add(beta));
			tangent = swapped ? new Tangent(poseOnC2, poseOnC1) : new Tangent(poseOnC1, poseOnC2);
		} else {
			Angle alpha = Angle.rad(Math.asin((this.getRadius() + other.getRadius()) / this.getDistance(other)));
			Angle beta = Angle.ANGLE_90.subtract(alpha);
			Angle gamma = new Pose2D(this.getX(), this.getY())
								  .getAngleTo(new Pose2D(other.getX(), other.getY()))
								  .subtract(beta);

			if (TangNo == 4) {
				gamma = gamma.add(beta);
				gamma = gamma.add(beta);
			}

			tangent = new Tangent(this.getPointOnCircle(gamma), other.getPointOnCircle(gamma.add(Angle.ANGLE_180)));
		}
		return tangent;
	}

	/**
	 * calculates the walked circle within three points
	 * @param p1 first point on circle
	 * @param p2 second point on circle
	 * @param p3 third point on circle
	 * @return new Circle2D(x, y, z) x & y = the middle of the circle z =
	 *         represents radiant of the circle
	 * @throws Exception
	 */
	public static Circle2D createWithThreePoints(Pose2D p1, Pose2D p2, Pose2D p3) throws Exception
	{
		if (p1.equals(p2) || p1.equals(p3) || p2.equals(p3)) {
			throw new Exception("three different points are needed for circle calculation");
		}

		double h =
				(p3.getY() - p1.getY()) * (p2.getX() - p1.getX()) - (p2.getY() - p1.getY()) * (p3.getX() - p1.getX());
		double k = Math.pow(p3.getX(), 2) - Math.pow(p1.getX(), 2) + Math.pow(p3.getY(), 2) - Math.pow(p1.getY(), 2);
		double l = Math.pow(p2.getX(), 2) - Math.pow(p1.getX(), 2) + Math.pow(p2.getY(), 2) - Math.pow(p1.getY(), 2);
		double y = (k * (p2.getX() - p1.getX()) - l * (p3.getX() - p1.getX())) / 2 / h;
		double x;
		if (p1.getX() == p2.getX()) {
			x = (k - 2 * y * (p3.getY() - p1.getY())) / 2 / (p3.getX() - p1.getX());
		} else {
			x = (l - 2 * y * (p2.getY() - p1.getY())) / 2 / (p2.getX() - p1.getX());
		}
		double r = Math.sqrt(Math.pow(p1.getX() - x, 2) + Math.pow(p1.getY() - y, 2));

		if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(r)) {
			throw new Exception("no possible circle calculation");
		}

		return new Circle2D(x, y, r);
	}

	/**
	 * calculate difference between two angles
	 * @param start: start angle
	 * @param end: end angle
	 * @param clockwise: calculation direction
	 * @return difference between both angles
	 */
	public static Angle getArc(Angle start, Angle end, boolean clockwise)
	{
		return getArc(start.degreesPositive(), end.degreesPositive(), clockwise);
	}

	/**
	 * calculate difference between two angles
	 * @param start: start angle
	 * @param end: end angle
	 * @param clockwise: calculation direction
	 * @return difference between both angles
	 */
	public static Angle getArc(double start, double end, boolean clockwise)
	{
		if (clockwise) {
			double value = 360 - (end - start);
			return Angle.deg(value);
		} else {
			double value = (end - start);
			return Angle.deg(value);
		}
	}
}
