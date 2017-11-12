package taco.agent.communication.perception;

public interface IJuryPerceptor {
	/**
	 * @return the global id of the maneuver to perform
	 */
	int getManeuverId();

	/**
	 * @return the action to perform
	 */
	JuryAction getAction();
}