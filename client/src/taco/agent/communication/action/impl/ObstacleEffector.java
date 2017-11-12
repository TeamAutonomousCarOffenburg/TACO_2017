package taco.agent.communication.action.impl;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.EffectorName;

public class ObstacleEffector implements IEffector
{
	private double posX;
	private double posY;

	public ObstacleEffector(double posX, double posY)
	{
		this.posX = posX;
		this.posY = posY;
	}

	@Override
	public String getName()
	{
		return EffectorName.OBSTACLE;
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
