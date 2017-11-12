package taco.agent.communication.perception.impl;

import taco.agent.communication.perception.IEnvironmentConfigPerceptor;
import taco.agent.communication.perception.PerceptorName;
import taco.agent.model.worldmodel.impl.ParkingSpace;
import taco.agent.model.worldmodel.signdetection.RoadSign;

import java.util.List;

public class EnvironmentConfigPerceptor extends AudiCupPerceptor implements IEnvironmentConfigPerceptor
{
	private List<RoadSign> roadSigns;

	private List<ParkingSpace> parkingSpaces;

	public EnvironmentConfigPerceptor(long timestamp, List<RoadSign> roadSigns, List<ParkingSpace> parkingSpaces)
	{
		super(PerceptorName.ENVIRONMENT_CONFIGURATION, timestamp);
		this.roadSigns = roadSigns;
		this.parkingSpaces = parkingSpaces;
	}

	@Override
	public List<RoadSign> getRoadSigns()
	{
		return roadSigns;
	}

	@Override
	public List<ParkingSpace> getParkingSpaces()
	{
		return parkingSpaces;
	}
}
