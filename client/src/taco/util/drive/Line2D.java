package taco.util.drive;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;

public class Line2D implements IBehaviorGeometry
{
	private Vector2D start, extension;

	public Line2D()
	{
		start = new Vector2D(0, 0);
		extension = new Vector2D(1, 0);
	}

	public Line2D(double m, double b)
	{
		start = new Vector2D(0, b);
		extension = new Vector2D(1, m);
	}

	public Line2D(double xStart, double yStart, Angle alpha, double length)
	{
		start = new Vector2D(xStart, yStart);
		extension = AngleUtils.getVectorFromAngle(alpha.radians(), length);
		if (extension.getX() == 0 && extension.getY() == 0) {
			extension = new Vector2D(1, 0);
		}
	}

	public Line2D(Vector2D start, Angle alpha, double length)
	{
		this.start = start;
		extension = AngleUtils.getVectorFromAngle(alpha.radians(), length);
		validate();
	}

	public Line2D(Vector2D start, Vector2D end)
	{
		this.start = start;
		extension = end.subtract(start);
		validate();
	}

	public Line2D(Vector3D start, Vector3D end)
	{
		this.start = new Vector2D(start.getX(), start.getY());
		Vector2D end2D = new Vector2D(end.getX(), end.getY());
		extension = end2D.subtract(this.start);
		validate();
	}

	public Line2D(double xStart, double yStart, double xEnd, double yEnd)
	{
		start = new Vector2D(xStart, yStart);
		extension = new Vector2D(xEnd - xStart, yEnd - yStart);
		validate();
	}

	public void validate()
	{
		if (extension.getX() == 0 && extension.getY() == 0) {
			extension = new Vector2D(0, 1);
		}
	}

	public Vector2D getStart()
	{
		return start;
	}

	public Vector2D getExtensionVector()
	{
		return extension;
	}

	public Vector2D getEnd()
	{
		return start.add(extension);
	}

	public double m()
	{
		if (extension.getX() == 0) {
			if (extension.getY() > 0) {
				return Double.MAX_VALUE;
			} else {
				return Double.MIN_VALUE;
			}
		}

		return extension.getY() / extension.getX();
	}

	public Angle getAngle()
	{
		return Angle.rad(Math.atan2(extension.getY(), extension.getX()));
	}

	public double b()
	{
		if (extension.getX() == 0) {
			return 0;
		}
		return start.getY() - ((extension.getY() / extension.getX()) * start.getX());
	}

	public double yValue(double x)
	{
		if (extension.getX() == 0) {
			// this line is y-parallel
			return 0;
		}

		return (extension.getY() / extension.getX()) * (x - start.getX()) + start.getY();
	}

	public double xValue(double y)
	{
		if (extension.getY() == 0) {
			// this line is x-parallel
			return start.getX();
		}

		return (extension.getX() / extension.getY()) * (y - start.getY()) + start.getX();
	}

	public IPose2D getClosestPose(Vector2D point)
	{
		Angle lineAngle = AngleUtils.to(extension);
		Angle pointAngle = AngleUtils.to(point.subtract(start));
		Angle offsetAngle = pointAngle.subtract(lineAngle);
		double hypotenuse = getDistance(point.getX(), point.getY(), start.getX(), start.getY());
		double factor = Math.cos(offsetAngle.radians()) * hypotenuse;

		double tmpExtNorm = Math.sqrt(extension.getX() * extension.getX() + extension.getY() * extension.getY());
		Vector2D tmp = new Vector2D(extension.getX() / tmpExtNorm * factor, extension.getY() / tmpExtNorm * factor);
		Vector2D closestPoint = start.add(tmp);

		return new Pose2D(closestPoint, AngleUtils.to(extension));
	}

	private double getDistance(double p1x, double p1y, double p2x, double p2y)
	{
		double xDiff = p2x - p1x;
		double yDiff = p2y - p1y;
		return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
	}

	public boolean isSame(Line2D other)
	{
		return (start == other.start && extension == other.extension);
	}

	@Override
	public IPose2D getClosestPose(Vector3D virtualPoint)
	{
		return this.getClosestPose(new Vector2D(virtualPoint.getX(), virtualPoint.getY()));
	}
}
