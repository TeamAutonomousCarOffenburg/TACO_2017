package taco.agent.model.agentmodel.impl;

import hso.autonomy.agent.communication.action.IEffector;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.ObstacleEffector;
import taco.agent.model.agentmodel.IObstacleActuator;

import java.util.Map;

public class ObstacleActuator extends AudiCupActuator implements IObstacleActuator
{
	Vector2D obstacle;

	public ObstacleActuator(String name)
	{
		super(name);
		obstacle = Vector2D.ZERO;
	}

	@Override
	public void setObstacle(Vector2D obstacle)
	{
		this.obstacle = obstacle;
	}

	@Override
	public boolean createAction(IAudiCupAction action, Map<String, IEffector> effectors)
	{
		if (obstacle == null) {
			return false;
		}
		effectors.put(getName(), new ObstacleEffector(obstacle.getX(), obstacle.getY()));
		obstacle = null;
		return true;
	}
}
