package taco.agent.communication.action.impl;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.EffectorName;
import taco.agent.model.worldmodel.impl.ParkingSpace;

public class ParkingSpaceEffector implements IEffector
{
	private int id;

	private double posX;

	private double posY;

	private int state;

	public ParkingSpaceEffector(ParkingSpace parkingSpace)
	{
		super();
		if (parkingSpace != null) {
			this.id = parkingSpace.getID();
			this.posX = parkingSpace.getPose().getX();
			this.posY = parkingSpace.getPose().getY();
			this.state = parkingSpace.getState().ordinal();
		}
	}

	@Override
	public String getName()
	{
		return EffectorName.PARKING_SPACE;
	}

	@Override
	public void setEffectorValues(float maxGain, float... values)
	{
	}

	@Override
	public void resetAfterAction()
	{
	}
}
