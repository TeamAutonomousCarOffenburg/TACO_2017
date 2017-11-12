package taco.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public interface IObstacleActuator extends IAudiCupActuator {
	void setObstacle(Vector2D obstacle);
}
