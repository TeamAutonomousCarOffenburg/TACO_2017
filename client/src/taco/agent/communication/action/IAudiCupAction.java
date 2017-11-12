package taco.agent.communication.action;

import java.util.Map;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.action.IEffector;

public interface IAudiCupAction extends IAction {
	/**
	 * Sets the map of effectors of this action
	 * @param effectors map of effectors
	 */
	void setEffectors(Map<String, IEffector> effectors);

	/**
	 * @param name the name of the effector to return
	 * @return the effector with corresponding name, null if none such exists
	 */
	IEffector getEffector(String name);
}