package taco.agent.model.agentmodel;

public interface ILight extends IAudiCupActuator {
	/**
	 * Turns on this light
	 */
	void turnOn();

	/**
	 * Turns off this light
	 */
	void turnOff();

	boolean isOn();
}