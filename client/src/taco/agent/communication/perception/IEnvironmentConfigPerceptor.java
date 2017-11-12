package taco.agent.communication.perception;

import hso.autonomy.agent.communication.perception.IPerceptor;
import taco.agent.model.worldmodel.impl.ParkingSpace;
import taco.agent.model.worldmodel.signdetection.RoadSign;

import java.util.List;

public interface IEnvironmentConfigPerceptor extends IPerceptor {
	List<RoadSign> getRoadSigns();
	List<ParkingSpace> getParkingSpaces();
}
