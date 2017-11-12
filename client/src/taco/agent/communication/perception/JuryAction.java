package taco.agent.communication.perception;

import com.google.gson.annotations.SerializedName;

public enum JuryAction {
	@SerializedName("-1") STOP,

	@SerializedName("0") GET_READY,

	@SerializedName("1") START
}
