package taco.agent.model.agentmodel;

import hso.autonomy.util.geometry.IPose2D;

public interface IPositionActuator extends IAudiCupActuator {
	void setCarPose(IPose2D carPose);
}
