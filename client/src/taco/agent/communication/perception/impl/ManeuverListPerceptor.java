package taco.agent.communication.perception.impl;

import java.util.List;

import taco.agent.communication.perception.IManeuverListPerceptor;
import taco.agent.communication.perception.PerceptorName;

public class ManeuverListPerceptor extends AudiCupPerceptor implements IManeuverListPerceptor
{
	private List<ManeuverPerceptor> maneuver;

	public ManeuverListPerceptor(long timestamp, List<ManeuverPerceptor> maneuver)
	{
		super(PerceptorName.MANEUVER_LIST, timestamp);
		this.maneuver = maneuver;
	}

	@Override
	public List<ManeuverPerceptor> getManeuver()
	{
		return maneuver;
	}
}
