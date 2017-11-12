package taco.agent.model.worldmodel;

/**
 * Constants used by drive instruction module
 */
public enum DriveInstruction {
	LEFT("left"),

	STRAIGHT("straight"),

	RIGHT("right"),

	PULL_OUT_LEFT("pull_out_left"),

	PULL_OUT_RIGHT("pull_out_right"),

	CROSS_PARKING("cross_parking"),

	PARALLEL_PARKING("parallel_parking"),

	// this one is added by us to indicate reactive driving
	FOLLOW_LANE("follow_lane"),

	// useful for driving in a circle endlessly
	STRAIGHT_FOREVER("straight_forever");

	/** the drive instruction name as it's used in the XML */
	public final String serializedName;

	DriveInstruction(String serializedName)
	{
		this.serializedName = serializedName;
	}

	public static DriveInstruction fromString(String text)
	{
		for (DriveInstruction instruction : DriveInstruction.values()) {
			if (instruction.serializedName.equalsIgnoreCase(text)) {
				return instruction;
			}
		}
		return null;
	}
}