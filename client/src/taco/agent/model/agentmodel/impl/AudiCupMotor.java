package taco.agent.model.agentmodel.impl;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.DoubleEffector;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmodel.IAudiCupMotor;

/**
 * Representation of the driving motor actuator
 */
public class AudiCupMotor extends ValueActuator<Double> implements IAudiCupMotor
{
	private boolean braking;

	private float brakingTime;

	/**
	 * @param name the name of the specific motor
	 */
	public AudiCupMotor(String name)
	{
		super(name);
		value = 0.0;
		previousValue = 0.0;
		braking = false;
		brakingTime = 0.0f;
	}

	@Override
	protected void setValue(Double value)
	{
		if (Math.abs(previousValue) > Math.abs(value)) {
			braking = true;
			brakingTime = currentTime;
		}

		super.setValue(value);
	}

	@Override
	public boolean update(IAudiCupPerception perception)
	{
		boolean result = super.update(perception);

		if (braking && currentTime - brakingTime > 1) {
			braking = false;
		}

		return result;
	}

	@Override
	public void driveForward()
	{
		drive(DEFAULT_SPEED);
	}

	@Override
	public void driveBackward()
	{
		drive(-DEFAULT_SPEED);
	}

	@Override
	public void drive(double speed)
	{
		setValue(speed);
	}

	@Override
	public void stop()
	{
		setValue(0.0);
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		effectors.put(getName(), new DoubleEffector(getName(), value));
		return true;
	}

	@Override
	public double getTargetSpeed()
	{
		return value;
	}

	@Override
	public boolean isBraking()
	{
		return braking;
	}
}
