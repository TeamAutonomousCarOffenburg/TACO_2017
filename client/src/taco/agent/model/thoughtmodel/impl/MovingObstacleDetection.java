package taco.agent.model.thoughtmodel.impl;

import hso.autonomy.util.geometry.Angle;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;

public class MovingObstacleDetection extends Detection
{
	@Override
	public void update(IAudiCupThoughtModel thoughtModel)
	{
		IAudiCupAgentModel agentModel = thoughtModel.getAgentModel();
		double speed = agentModel.getMotor().getTargetSpeed();
		double distance = 1;
		double obstacleDistance = thoughtModel.getObstacleAheadDistance(distance);

		setValidity(speed > 20 && obstacleDistance < distance, thoughtModel.getWorldModel().getGlobalTime());
	}
}
