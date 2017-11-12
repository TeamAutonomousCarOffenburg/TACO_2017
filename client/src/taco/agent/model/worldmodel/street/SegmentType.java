package taco.agent.model.worldmodel.street;

/**
 * Types of segments we know
 */
public enum SegmentType {
	T_CROSSING(1, 1),
	X_CROSSING(1, 1),
	STRAIGHT(1, 1),
	// technically not a separate type (at least it's not listed in the AADC manual)
	STRAIGHT_WITH_CROSSWALK(1, 1),
	CURVE_SMALL(2, 2),
	CURVE_BIG(3, 3),
	S_CURVE_TOP(3, 3),
	S_CURVE_BOTTOM(3, 3),
	PARKING_SPACE_VERTICAL(2, 1),
	// not used in AADC2017
	PARKING_SPACE_HORIZONTAL(4, 1);

	public final int width;

	public final int height;

	SegmentType(int width, int height)
	{
		this.width = width;
		this.height = height;
	}

	public boolean isParkingSpace()
	{
		return this == PARKING_SPACE_VERTICAL || this == SegmentType.PARKING_SPACE_HORIZONTAL;
	}
}
