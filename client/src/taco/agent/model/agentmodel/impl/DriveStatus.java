package taco.agent.model.agentmodel.impl;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.CarState;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.DriveStatusEffector;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmodel.IDriveStatus;

/**
 * Actuator for drive status
 */
public class DriveStatus extends AudiCupActuator implements IDriveStatus
{
	private int maneuverId;

	private CarState status;

	public DriveStatus(String name)
	{
		super(name);
		maneuverId = 0;
		status = CarState.STARTUP;
	}

	@Override
	public void setManeuverId(int maneuverId)
	{
		this.maneuverId = maneuverId;
	}

	@Override
	public void setStatus(CarState status)
	{
		this.status = status;
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		effectors.put(getName(), new DriveStatusEffector(maneuverId, status.getValue()));
		return true;
	}
}
