package taco.agent.model.worldmodel.street;

import hso.autonomy.util.geometry.Angle;

public enum Direction {
	NORTH,
	EAST,
	SOUTH,
	WEST;

	/**
	 * @param theAngle the angle for which to get the direction
	 * @return a global direction the angle is facing to
	 */
	public static Direction getDirection(Angle theAngle)
	{
		double angle = theAngle.degrees();
		if (angle >= -45 && angle < 45) {
			return Direction.NORTH;
		} else if (angle >= 45 && angle < 135) {
			return Direction.WEST;
		} else if (angle >= -135 && angle < -45) {
			return Direction.EAST;
		}
		return Direction.SOUTH;
	}

	/**
	 * @param dir a global direction
	 * @return the opposite direction as the passed one
	 */
	public static Direction getOppositeDirection(Direction dir)
	{
		return Direction.values()[(dir.ordinal() + 2) % 4];
	}

	public Angle getAngle()
	{
		switch (this) {
		case NORTH:
			return Angle.ZERO;
		case EAST:
			return Angle.ANGLE_90.negate();
		case SOUTH:
			return Angle.ANGLE_180;
		case WEST:
		default:
			return Angle.ANGLE_90;
		}
	}

	/**
	 * @param inDirection global direction from which we come
	 * @param outDirection direction local to inDirection
	 * @return the global direction from the specified directions, i.e. if inDirection is North and outDirection is
	 * East, we return West
	 */
	public static Direction getAbsoluteFromRelativeDirection(Direction inDirection, Direction outDirection)
	{
		return Direction.values()[((inDirection.ordinal() + outDirection.ordinal()) % 4)];
	}

	/**
	 * @param dir the global direction we are facing
	 * @return the direction left of this direction
	 */
	public static Direction getLeft(Direction dir)
	{
		return Direction.values()[(4 + dir.ordinal() - 1) % 4];
	}

	/**
	 * @param dir the global direction we are facing
	 * @return the direction right of this direction
	 */
	public static Direction getRight(Direction dir)
	{
		return Direction.values()[(dir.ordinal() + 1) % 4];
	}

	/**
	 * @return the next direction as of the natural order of directions (N-E-S-W)
	 */
	public Direction getNext()
	{
		return Direction.values()[(ordinal() + 1) % 4];
	}
}