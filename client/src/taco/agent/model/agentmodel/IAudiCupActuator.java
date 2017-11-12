package taco.agent.model.agentmodel;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.perception.IAudiCupPerception;

/**
 * Interface for all actuators (motors, steering, light)
 */
public interface IAudiCupActuator {
	String getName();

	boolean isInitialized();

	boolean update(IAudiCupPerception perception);

	boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors);

	float getModificationTime();
}
