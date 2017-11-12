package taco.agent.communication.perception;

import taco.agent.model.worldmodel.signdetection.RoadSign;

import java.util.List;

public interface ISignPerceptor {
	List<RoadSign> getSigns();
}
