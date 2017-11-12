/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

public class Arc2D
{
	/** x coordinate of the circle center */
	private double x;

	/** y coordinate of the circle center */
	private double y;

	/** radius of the circle */
	private double radius;

	/** the angle where to start the arc */
	private Angle startAngle;

	/** the angle where to end the arc */
	private Angle endAngle;

	public Arc2D(double x, double y, double radius, Angle startAngle, Angle endAngle)
	{
		this.x = x;
		this.y = y;
		this.radius = radius;
		this.startAngle = startAngle;
		this.endAngle = endAngle;
	}

	public double getX()
	{
		return x;
	}

	public double getY()
	{
		return y;
	}

	public double getRadius()
	{
		return radius;
	}

	public Angle getStartAngle()
	{
		return startAngle;
	}

	public Angle getEndAngle()
	{
		return endAngle;
	}
}
