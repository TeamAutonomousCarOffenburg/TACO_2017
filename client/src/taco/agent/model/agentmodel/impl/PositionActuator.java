package taco.agent.model.agentmodel.impl;

import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.PositionEffector;
import taco.agent.model.agentmodel.IPositionActuator;

import java.util.Map;

public class PositionActuator extends AudiCupActuator implements IPositionActuator
{
	private IPose2D carPose;

	public PositionActuator(String name)
	{
		super(name);
		carPose = new Pose2D(Vector2D.ZERO, Angle.ZERO);
	}

	@Override
	public void setCarPose(IPose2D carPose)
	{
		this.carPose = carPose;
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		effectors.put(getName(), new PositionEffector(carPose.getX(), carPose.getY(), carPose.getAngle().radians()));
		return true;
	}
}
