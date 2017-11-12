package taco.util.drive;

import hso.autonomy.util.misc.ValueUtil;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;

/*
 * Class collecting needed Angle methods from c++.
 */
public class AngleUtils
{
	/**
	 * Returns a 2D-Vector from a radian value that represents an Angle.
	 */
	public static Vector2D getVectorFromAngle(double radAngle, double length)
	{
		return new Vector2D(Math.cos(radAngle) * length, Math.sin(radAngle) * length);
	}

	public static Vector3D get3DVectorFromAngle(double radAngle, double length)
	{
		return new Vector3D(Math.cos(radAngle) * length, Math.sin(radAngle) * length, 0);
	}

	public static Angle oppositeAngle(Angle radAngle)
	{
		return radAngle.add(Angle.ANGLE_180);
	}

	public static Angle to(Vector2D point)
	{
		if (point.getX() == 0 && point.getY() == 0) {
			return Angle.ZERO;
		}

		return Angle.rad((Math.atan2(point.getY(), point.getX())));
	}

	public static Angle to(Vector3D point)
	{
		if (point.getX() == 0 && point.getY() == 0) {
			return Angle.ZERO;
		}

		return Angle.rad((Math.atan2(point.getY(), point.getX())));
	}

	public static Angle to(double x, double y)
	{
		if (x == 0 && y == 0) {
			return Angle.ZERO;
		}

		return Angle.rad((Math.atan2(y, x)));
	}

	public static Vector3D rotate(Vector3D vec, double radAngle)
	{
		double cosAngle = Math.cos(radAngle);
		double sinAngle = Math.sin(radAngle);

		// Calculate new position
		Vector3D res = new Vector3D(
				cosAngle * vec.getX() - sinAngle * vec.getY(), sinAngle * vec.getX() + cosAngle * vec.getY(), 0);
		return res;
	}

	public static Angle limit(Angle angle, Angle min, Angle max)
	{
		return Angle.deg(ValueUtil.limitValue(angle.degrees(), min.degrees(), max.degrees()));
	}
}
