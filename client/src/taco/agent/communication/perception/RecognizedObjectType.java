package taco.agent.communication.perception;

import com.google.gson.annotations.SerializedName;

public enum RecognizedObjectType {
	@SerializedName("1")
	PERSON_CROSSING,

	@SerializedName("2")
	PERSON,

	@SerializedName("3")
	CHILD,

	@SerializedName("4")
	STOP_LINE_LEFT,

	@SerializedName("5")
	STOP_LINE_RIGHT,

	@SerializedName("6")
	STOP_LINE_AHEAD,

	@SerializedName("7")
	STOP_LINE_ONCOMING,

	@SerializedName("8")
	CAR,

	@SerializedName("9")
	CROSSWALK,

	@SerializedName("10")
	MIDDLE_LANE;

	public boolean isPedestrian()
	{
		return this == PERSON || this == PERSON_CROSSING || this == CHILD;
	}

	public boolean isObjectForBackend()
	{
		return isPedestrian() || this == CAR;
	}
}
