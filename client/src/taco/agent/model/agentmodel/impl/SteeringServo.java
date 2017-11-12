package taco.agent.model.agentmodel.impl;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.util.geometry.Angle;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.DoubleEffector;
import taco.agent.model.agentmodel.ISteeringServo;

/**
 * Representation of the steering of the car
 */
public class SteeringServo extends ValueActuator<Double> implements ISteeringServo
{
	/**
	 * @param name the name of the specific steering
	 */
	public SteeringServo(String name)
	{
		super(name);
		value = 0.0;
		previousValue = 0.0;
	}

	@Override
	public void steer(Angle angle)
	{
		setValue(angle.degrees());
	}

	@Override
	public void reset()
	{
		steer(Angle.ZERO);
	}

	@Override
	public Angle getDesiredAngle()
	{
		return Angle.deg(value);
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		effectors.put(getName(), new DoubleEffector(getName(), value));
		return true;
	}
}
