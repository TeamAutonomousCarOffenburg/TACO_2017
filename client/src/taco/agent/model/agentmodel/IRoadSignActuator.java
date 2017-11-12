package taco.agent.model.agentmodel;

import taco.agent.model.worldmodel.signdetection.RoadSign;

public interface IRoadSignActuator extends IAudiCupActuator {
	void setRoadSign(RoadSign roadSign);
}
