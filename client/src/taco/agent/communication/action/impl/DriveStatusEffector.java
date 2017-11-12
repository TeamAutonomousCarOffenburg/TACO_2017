package taco.agent.communication.action.impl;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.EffectorName;

public class DriveStatusEffector implements IEffector
{
	private int maneuverId;

	private int status;

	public DriveStatusEffector(int maneuverId, int status)
	{
		super();
		this.maneuverId = maneuverId;
		this.status = status;
	}

	@Override
	public String getName()
	{
		return EffectorName.DRIVE_STATUS;
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
