/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import java.util.Locale;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * Stores a 2D position and orientation.
 */
public class Pose2D implements IPose2D
{
	/** The x-position value */
	public double x;

	/** The y-position value */
	public double y;

	/** The horizontal angle. */
	public Angle angle;

	public Pose2D(double x, double y, Angle angle)
	{
		this.x = x;
		this.y = y;
		this.angle = angle;
	}

	public Pose2D(double x, double y)
	{
		this(x, y, Angle.ZERO);
	}

	public Pose2D(Vector2D position, Angle angle)
	{
		this(position.getX(), position.getY(), angle);
	}

	public Pose2D(Vector2D position)
	{
		this(position, Angle.ZERO);
	}

	public Pose2D(Vector3D position, Angle angle)
	{
		this(position.getX(), position.getY(), angle);
	}

	public Pose2D(Vector3D position)
	{
		this(position, Angle.ZERO);
	}

	/**
	 * Default constructor initializing all values with zero.
	 */
	public Pose2D()
	{
		this(0, 0, Angle.ZERO);
	}

	/**
	 * Copy constructor.
	 */
	public Pose2D(Pose2D other)
	{
		this(other.x, other.y, other.angle);
	}

	public void copy(Pose2D other)
	{
		this.x = other.x;
		this.y = other.y;
		this.angle = other.angle;
	}

	@Override
	public double getX()
	{
		return x;
	}

	@Override
	public double getY()
	{
		return y;
	}

	@Override
	public Vector3D getPosition()
	{
		return new Vector3D(x, y, 0);
	}

	@Override
	public Angle getAngle()
	{
		return angle;
	}

	public void setPosition(Vector3D position)
	{
		this.x = position.getX();
		this.y = position.getY();
	}

	/**
	 * Calculate the average of a list of Pose2D.
	 *
	 * @param input: array of input values
	 * @return calculated average
	 */
	public static Pose2D average(Pose2D[] input)
	{
		// float angle = averageAngle(angles);
		double avgX = 0.0;
		double avgY = 0.0;

		// average position
		for (Pose2D value : input) {
			avgX += value.x;
			avgY += value.y;
		}
		if (input.length > 1) {
			avgX /= input.length;
			avgY /= input.length;
		}

		// average angles
		Angle[] angles = new Angle[input.length];
		for (int i = 0; i < input.length; i++) {
			angles[i] = input[i].angle;
		}

		return new Pose2D(avgX, avgY, Angle.average(angles));
	}

	@Override
	public Pose2D applyTo(IPose2D other)
	{
		Vector3D rotPos = angle.applyTo(other.getX(), other.getY());

		return new Pose2D(x + rotPos.getX(), y + rotPos.getY(), angle.add(other.getAngle()));
	}

	@Override
	public Pose2D applyInverseTo(IPose2D other)
	{
		Vector3D rotPos = angle.applyInverseTo(other.getX() - x, other.getY() - y);

		return new Pose2D(rotPos, other.getAngle().subtract(angle));
	}

	@Override
	public Vector3D applyTo(Vector3D position)
	{
		Vector3D rotPos = angle.applyTo(position);

		return new Vector3D(x + rotPos.getX(), y + rotPos.getY(), position.getZ());
	}

	@Override
	public Vector2D applyTo(Vector2D position)
	{
		Vector2D rotPos = angle.applyTo(position);

		return new Vector2D(x + rotPos.getX(), y + rotPos.getY());
	}

	@Override
	public Vector3D applyInverseTo(Vector3D position)
	{
		return angle.applyInverseTo(position.getX() - x, position.getY() - y);
	}

	@Override
	public double getDistanceTo(IPose2D other)
	{
		double deltaX = x - other.getX();
		double deltaY = y - other.getY();
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}

	@Override
	public Angle getDeltaAngle(IPose2D other)
	{
		return other.getAngle().subtract(angle);
	}

	@Override
	public Angle getAngleTo(IPose2D other)
	{
		Pose2D relative = applyInverseTo(other);
		return Angle.rad(relative.getPosition().getAlpha());
	}

	@Override
	public Vector3D getUnitVector()
	{
		return new Vector3D(angle.radians(), 0);
	}

	@Override
	public Pose2D flip()
	{
		return new Pose2D(x * -1, y * -1, angle.add(Angle.ANGLE_180));
	}

	@Override
	public String toString()
	{
		return String.format(Locale.US, "(%.2f, %.2f; %s)", x, y, angle);
	}

	@Override
	public boolean equals(Object other)
	{
		if (!(other instanceof Pose2D))
			return false;
		Pose2D p = (Pose2D) other;
		return (this.x == p.x && this.y == p.y && this.angle.equals(p.angle));
	}

	public static Pose2D average(IPose2D p1, IPose2D p2, int weight1, int weight2)
	{
		double divider = weight1 + weight2;
		double x = (p1.getX() * weight1 + p2.getX() * weight2) / divider;
		double y = (p1.getY() * weight1 + p2.getY() * weight2) / divider;

		return new Pose2D(x, y, Angle.average(p1.getAngle(), p2.getAngle(), weight1, weight2));
	}
}
