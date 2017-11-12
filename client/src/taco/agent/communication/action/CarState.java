package taco.agent.communication.action;

public enum CarState {
	STARTUP(-2),
	ERROR(-1),
	READY(0),
	RUNNING(1),
	COMPLETE(2);

	private final int value;

	CarState(int value)
	{
		this.value = value;
	}

	public int getValue()
	{
		return value;
	}
}
