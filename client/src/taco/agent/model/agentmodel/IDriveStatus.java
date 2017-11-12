package taco.agent.model.agentmodel;

import taco.agent.communication.action.CarState;

public interface IDriveStatus extends IAudiCupActuator {
	/**
	 * @param maneuverId the id of the current maneuver that is performed
	 */
	void setManeuverId(int maneuverId);

	/**
	 * @param status of the maneuver (see Audi Cup specifications)
	 */
	void setStatus(CarState status);
}