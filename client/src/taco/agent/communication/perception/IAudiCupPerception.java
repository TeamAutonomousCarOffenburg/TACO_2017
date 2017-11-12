package taco.agent.communication.perception;

import hso.autonomy.agent.communication.perception.IPerception;
import taco.agent.communication.perception.impl.LaneMiddlePerceptor;

public interface IAudiCupPerception extends IPerception {
	IWheelTickPerceptor getWheelTickPerceptor(String name);

	IImuPerceptor getCarImuPerceptor(String name);

	LaneMiddlePerceptor getLaneMiddlePerceptor(String name);

	IDoublePerceptor getDoublePerceptor(String name);

	IManeuverListPerceptor getManeuverListPerceptor();

	IJuryPerceptor getJuryPerceptor();

	ISignPerceptor getSignPerceptor();

	IFloorNormalPerceptor getFloorNormalPerceptor();

	IVisionPerceptor getVisionPerceptor();

	IEnvironmentConfigPerceptor getEnvironmentPerceptor();
}