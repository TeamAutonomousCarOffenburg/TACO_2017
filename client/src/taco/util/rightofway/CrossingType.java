package taco.util.rightofway;

/**
 * The relative type of a crossing assuming we see the crossing coming from south direction
 */
public enum CrossingType {
	/** X-crossing */
	NORTH_EAST_SOUTH_WEST,

	/** T-crossing with east, south and west exits */
	EAST_SOUTH_WEST,

	/** T-crossing with north, east and south exits */
	NORTH_EAST_SOUTH,

	/** T-crossing with north, south and west exits */
	NORTH_SOUTH_WEST,
}
