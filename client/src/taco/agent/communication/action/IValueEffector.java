package taco.agent.communication.action;

import hso.autonomy.agent.communication.action.IEffector;

/**
 * Interface for effectors that only have a double value
 */
public interface IValueEffector<T> extends IEffector {
	void setValue(T value);

	T getValue();
}
