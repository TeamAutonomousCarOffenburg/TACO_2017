package hso.autonomy.util.geometry;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Line2D
{
	/** The start point of the line segment. */
	private Vector2D start;

	/** The line segment extension. */
	private Vector2D extension;

	/** Construct a line from point (0, 0) to (1, 0) */
	public Line2D()
	{
		this.start = new Vector2D(0, 0);
		this.extension = new Vector2D(1, 0);
	}

	/** Construct a line with the given slope and y-offset
	 * @param m - the slope of the line
	 * @param b - the y-offset of the line
	 */
	public Line2D(final double m, final double b)
	{
		this.start = new Vector2D(0, b);
		this.extension = new Vector2D(1, m);
	}

	/** Construct a line with a start point, and angle and length of the line.
	 * @param xStart - the x-coordinate of the start point (or an arbitrary point on the line)
	 * @param yStart - the y-coordinate of the start point (or an arbitrary point on the line)
	 * @param alpha - the angle from the x-axis to the line
	 * @param length - the line segment length (or any value > 0)
	 */
	public Line2D(final double xStart, final double yStart, final Angle alpha, final double length)
	{
		this.start = new Vector2D(xStart, yStart);
		this.extension = new Vector2D(Math.cos(alpha.radians()) * length, Math.sin(alpha.radians()) * length);
		validate();
	}

	/** Construct a line with a start point, and angle with a length of 1.
	 * @param xStart - the x-coordinate of the start point (or an arbitrary point on the line)
	 * @param yStart - the y-coordinate of the start point (or an arbitrary point on the line)
	 * @param alpha - the angle from the x-axis to the line
	 */
	public Line2D(final double xStart, final double yStart, final Angle alpha)
	{
		this(xStart, yStart, alpha, 1);
	}

	/** Construct a line with a start point, and angle and length of the line.
	 * @param start - the start point of the line segment (or an arbitrary point on the line)
	 * @param alpha - the angle from the x-axis to the line
	 * @param length - the line segment length (or any value > 0)
	 */
	public Line2D(final Vector2D start, final Angle alpha, final double length)
	{
		this.start = start;
		this.extension = new Vector2D(Math.cos(alpha.radians()) * length, Math.sin(alpha.radians()) * length);
		validate();
	}

	/** Construct a line with a start point, and angle with a length of 1.
	 * @param start - the start point of the line segment (or an arbitrary point on the line)
	 * @param alpha - the angle from the x-axis to the line
	 */
	public Line2D(final Vector2D start, final Angle alpha)
	{
		this(start, alpha, 1);
	}

	/** Construct a line with a start and end point (or two arbitrary points on a the line).
	 * @param start - the start point of the line segment
	 * @param end - the end point of the line segment
	 */
	public Line2D(final Vector2D start, final Vector2D end)
	{
		this.start = start;
		this.extension = end.subtract(start);
		validate();
	}

	/** Construct a line with a start and end point (or two arbitrary points on a the line).
	 * @param xStart - the x-coordinate of the start point of the line segment
	 * @param yStart - the y-coordinate of the start point of the line segment
	 * @param xEnd - the x-coordinate of the end point of the line segment
	 * @param yEnd - the y-coordinate of the end point of the line segment
	 */
	public Line2D(final double xStart, final double yStart, final double xEnd, final double yEnd)
	{
		this.start = new Vector2D(xStart, yStart);
		this.extension = new Vector2D(xEnd - xStart, yEnd - yStart);
	}

	/** Retrieve the start point of the line segment. */
	public Vector2D getStart()
	{
		return this.start;
	}

	/** Retrieve the line segment extension vector. */
	public Vector2D getExtensionVector()
	{
		return this.extension;
	}
	/** Retrieve the end point of the line segment. */
	Vector2D getEnd()
	{
		return this.start.add(this.extension);
	}

	/** Retrieve the angle from the x-axis to this line. */
	public Angle getAngle()
	{
		return Angle.to(this.extension);
	}

	/** Retrieve the slope of this line (of line-formula: y = m*x + b). */
	public double m()
	{
		if (this.extension.getX() == 0) {
			if (this.extension.getY() > 0) {
				return Double.MAX_VALUE;
			} else {
				return Double.MIN_VALUE;
			}
		}

		return this.extension.getY() / this.extension.getX();
	}

	/** Retrieve the y-offset of this line (of line-formula: y = m*x + b). */
	public double b()
	{
		if (this.extension.getX() == 0) {
			return 0;
		}

		return this.start.getY() - ((this.extension.getY() / this.extension.getX()) * this.start.getX());
	}

	/** Retrieve the y-value of this line at the given x-value */
	public double yValue(double x)
	{
		if (this.extension.getX() == 0) {
			// this line is y-parallel
			return 0;
		}

		return (this.extension.getY() / this.extension.getX()) * (x - this.start.getX()) + this.start.getY();
	}

	/** Retrieve the x-value of this line at the given y-value */
	public double xValue(double y)
	{
		if (this.extension.getY() == 0) {
			// this line is x-parallel
			return this.start.getX();
		}

		return (this.extension.getX() / this.extension.getY()) * (y - this.start.getY()) + this.start.getX();
	}

	/** Retrieve the closest point and slope angle of this point on the line from given point */
	public Pose2D getClosestPose(final Vector2D point)
	{
		Angle lineAngle = Angle.to(this.extension);
		Angle pointAngle = Angle.to(point.subtract(this.start));

		Angle offsetAngle = pointAngle.subtract(lineAngle);
		double hypotenuse = point.distance(this.start);
		double factor = Math.cos(offsetAngle.radians()) * hypotenuse;
		Vector2D closestPoint = new Vector2D(this.extension.getX() / this.extension.getNorm() * factor,
				this.extension.getY() / this.extension.getNorm() * factor);
		closestPoint = this.start.add(closestPoint);

		return new Pose2D(closestPoint, Angle.to(this.extension));
	}

	/** Retrieve trail as vector of trail points */
	public List<Vector2D> getTrail(final Vector2D start, final Vector2D end)
	{
		Pose2D poseStart = getClosestPose(start);
		Pose2D poseEnd = getClosestPose(end);
		double distance = poseStart.getDistanceTo(poseEnd);
		double angle = poseStart.getAngle().radians();
		double length = 0.1 * distance;
		Vector2D offset = new Vector2D(Math.cos(angle) * length, Math.sin(angle) * length);
		Vector3D point3D = poseStart.getPosition();
		Vector2D point = new Vector2D(point3D.getX(), point3D.getY());
		ArrayList<Vector2D> points = new ArrayList<>();
		int i = 0;
		do {
			point = point.add(offset);
			points.add(point);
		} while (++i < 10);
		return points;
	}

	@Override
	public boolean equals(Object other)
	{
		Line2D otherLine = (Line2D) other;
		return this.start == otherLine.start && this.extension == otherLine.extension;
	}

	/** Validate that this instance represents a proper line. If not, make this line the x-axis. */
	private void validate()
	{
		if (this.extension.getX() == 0 && this.extension.getY() == 0) {
			this.extension = new Vector2D(0, 1);
		}
	}
}
