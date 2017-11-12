package taco.agent.communication.action.impl;

import hso.autonomy.agent.communication.action.IEffector;
import taco.agent.communication.action.EffectorName;
import taco.agent.model.worldmodel.signdetection.RoadSign;

public class RoadSignEffector implements IEffector
{
	private int id;

	private double posX;

	private double posY;

	private double angle;

	public RoadSignEffector(RoadSign roadSign)
	{
		super();
		if (roadSign != null) {
			this.id = roadSign.getSignType().getValue();
			this.posX = roadSign.getPose().getX();
			this.posY = roadSign.getPose().getY();
			this.angle = roadSign.getPose().getAngle().degrees();
		}
	}

	@Override
	public String getName()
	{
		return EffectorName.ROAD_SIGN;
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
