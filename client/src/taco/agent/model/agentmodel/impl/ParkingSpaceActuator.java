package taco.agent.model.agentmodel.impl;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.EffectorName;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.ParkingSpaceEffector;
import taco.agent.model.agentmodel.IParkingSpaceActuator;
import taco.agent.model.worldmodel.impl.ParkingSpace;

import java.util.Map;

public class ParkingSpaceActuator extends AudiCupActuator implements IParkingSpaceActuator
{
	private ParkingSpace parkingSpace;

	public ParkingSpaceActuator(String name)
	{
		super(name);
	}

	@Override
	public void setParkingSpace(ParkingSpace parkingSpace)
	{
		this.parkingSpace = parkingSpace;
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		if (parkingSpace == null) {
			return false;
		}
		effectors.put(getName(), new ParkingSpaceEffector(parkingSpace));
		parkingSpace = null;
		return true;
	}
}
