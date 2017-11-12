package taco.util.rightofway;

public class CarPositions
{
	/** Car coming from north */
	public final boolean north;

	/** Car coming from east */
	public final boolean east;

	/** Car coming from west */
	public final boolean west;

	public CarPositions(boolean north, boolean east, boolean west)
	{
		this.north = north;
		this.east = east;
		this.west = west;
	}
}
