package taco.util.drive;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;

public class Circle2D extends hso.autonomy.util.geometry.Circle2D implements IBehaviorGeometry
{
	private Vector3D origin;

	public Circle2D(double x, double y, double radius)
	{
		super(x, y, (radius >= 0 ? radius : -radius));
		origin = new Vector3D(0, 0, 0);
	}

	Circle2D(IPose2D tangent, Vector2D point)
	{
		this(tangent, new Vector3D(point.getX(), point.getY(), 0));
	}

	Circle2D(IPose2D tangent, Vector3D point)
	{
		// transform point in to an tangent touching point local coordinate system (tangent angle = x-axis + pi)
		Vector3D tmpVector = point.subtract(tangent.getPosition());
		Vector3D rotatedVector = AngleUtils.rotate(tmpVector, -1 * tangent.getAngle().radians());

		double my =
				(-Math.pow(rotatedVector.getX(), 2) - Math.pow(rotatedVector.getY(), 2)) / (-2 * rotatedVector.getY());

		// transform back to world coordinate system

		Vector3D shiftedCircleOrigin = AngleUtils.rotate(new Vector3D(0, my, 0), tangent.getAngle().radians());
		origin = shiftedCircleOrigin.add(tangent.getPosition());
		setRadius(getCircleDistance(origin, tangent.getPosition()));
		setX(origin.getX());
		setY(origin.getY());
	}

	private double getCircleDistance(Vector3D c1, Vector3D c2)
	{
		double xDiff = c2.getX() - c1.getX();
		double yDiff = c2.getY() - c1.getY();
		return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
	}

	public Angle slope(Vector3D vector3d)
	{
		// calculate slope on upper semicircle
		double slope = -0.5 * Math.sqrt(1 / (Math.pow(getRadius(), 2) - Math.pow(vector3d.getX() - origin.getX(), 2))) *
					   (2 * vector3d.getX() - 2 * origin.getX());

		// check if point is on lower semicircle
		if (origin.getY() > vector3d.getY()) {
			slope *= -1;
		}
		return Angle.rad(Math.atan(slope));
	}

	public int getDirectionOfRotation(IPose2D nextWaypoint)
	{
		return getDirectionOfRotation(nextWaypoint.getPosition(), nextWaypoint.getAngle());
	}

	public int getDirectionOfRotation(Vector3D point, Angle angle)
	{
		Angle angle2 = getAngle(point);
		angle2 = angle2.add(Angle.rad(Math.PI * 0.5));
		double angleResult = Math.abs(angle2.radians() - angle.radians()) + (Math.PI * 0.5);

		if (angleResult < Math.PI || angleResult > 2 * Math.PI) {
			return 1;
		} else {
			return -1;
		}
	}

	Angle getAngle(Vector3D point)
	{
		Vector3D shiftedPoint = point.subtract(origin);
		// point is equal to origin
		if (Math.abs(point.getX() - origin.getX()) < 0.001 && Math.abs(point.getY() - origin.getY()) < 0.001)
			return Angle.ZERO;
		// threshold x = 0
		if (Math.abs(shiftedPoint.getX()) < 0.0001) {
			if (shiftedPoint.getY() > 0) {
				return Angle.deg(180 * 0.5);
			} else
				return Angle.deg(180 * -0.5);
		}

		double angle = Math.atan2(shiftedPoint.getY(), shiftedPoint.getX());
		return Angle.rad(angle);
	}

	Angle getAngle(double segmentLength)
	{
		return Angle.rad(segmentLength / getRadius());
	}

	boolean isOnCircle(Vector3D point)
	{
		double distance = origin.distance(point);
		double diff = Math.abs(distance - getRadius());
		return (diff < 0.000001);
	}

	@Override
	public IPose2D getClosestPose(Vector2D virtualPoint)
	{
		return this.getClosestPose(new Vector3D(virtualPoint.getX(), virtualPoint.getY(), 0));
	}

	public IPose2D getClosestPose(Vector3D virtualPoint)
	{
		Angle angle = AngleUtils.to(virtualPoint.subtract(origin));
		Vector3D closestPoint = getPointOnCircle(angle).getPosition();
		return new Pose2D(closestPoint, slope(closestPoint));
	}

	public Vector3D getOrigin()
	{
		return origin;
	}

	public static Circle2D average(ArrayList<Circle2D> circles)
	{
		int size = circles.size();
		if (size == 0) {
			return new Circle2D(0, 0, 1);
		} else if (size == 1) {
			return circles.get(0);
		}

		double x = 0;
		double y = 0;
		double radius = 0;

		for (Circle2D circle : circles) {
			x += circle.getOrigin().getX();
			y += circle.getOrigin().getY();
			radius += circle.getRadius();
		}

		return new Circle2D(x / size, y / size, radius / size);
	}

	public Vector3D getVectorPointOnCircle(Angle angle)
	{
		return new Vector3D(origin.getX() + Math.cos(angle.radians()) * getRadius(),
				origin.getY() + Math.sin(angle.radians()) * getRadius(), 0);
	}
}
