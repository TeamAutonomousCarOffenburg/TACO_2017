package taco.agent.model.agentmodel.impl;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.BooleanEffector;
import taco.agent.model.agentmodel.ILight;

/**
 * Specific Light actuator
 */
public class Light extends ValueActuator<Boolean> implements ILight
{
	/**
	 * @param name the name of the specific light
	 */
	public Light(String name)
	{
		super(name);
		value = false;
		previousValue = false;
	}

	@Override
	public void turnOn()
	{
		setValue(true);
	}

	@Override
	public void turnOff()
	{
		setValue(false);
	}

	@Override
	public boolean isOn()
	{
		return value;
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		effectors.put(getName(), new BooleanEffector(getName(), value));
		return true;
	}
}
