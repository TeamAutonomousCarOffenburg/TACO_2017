package taco.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.IPerceptor;
import hso.autonomy.agent.communication.perception.impl.Perception;
import taco.agent.communication.perception.*;

import java.util.Map;

public class AudiCupPerception extends Perception implements IAudiCupPerception
{
	@Override
	public IWheelTickPerceptor getWheelTickPerceptor(String name)
	{
		return (IWheelTickPerceptor) perceptors.get(name);
	}

	@Override
	public IImuPerceptor getCarImuPerceptor(String name)
	{
		return (IImuPerceptor) perceptors.get(name);
	}

	@Override
	public LaneMiddlePerceptor getLaneMiddlePerceptor(String name)
	{
		return (LaneMiddlePerceptor) perceptors.get(name);
	}

	@Override
	public DoublePerceptor getDoublePerceptor(String name)
	{
		return (DoublePerceptor) perceptors.get(name);
	}

	@Override
	public IManeuverListPerceptor getManeuverListPerceptor()
	{
		return (IManeuverListPerceptor) perceptors.get(PerceptorName.MANEUVER_LIST);
	}

	@Override
	public IJuryPerceptor getJuryPerceptor()
	{
		return (IJuryPerceptor) perceptors.get(PerceptorName.JURY_COMMAND);
	}

	@Override
	public ISignPerceptor getSignPerceptor()
	{
		return (ISignPerceptor) perceptors.get(PerceptorName.SIGNS);
	}

	@Override
	public IFloorNormalPerceptor getFloorNormalPerceptor()
	{
		return (IFloorNormalPerceptor) perceptors.get(PerceptorName.FLOOR_NORMAL);
	}

	@Override
	public IVisionPerceptor getVisionPerceptor()
	{
		return (IVisionPerceptor) perceptors.get(PerceptorName.VISION);
	}

	@Override
	public IEnvironmentConfigPerceptor getEnvironmentPerceptor()
	{
		return (IEnvironmentConfigPerceptor) perceptors.get(PerceptorName.ENVIRONMENT_CONFIGURATION);
	}

	public Map<String, IPerceptor> getPerceptors()
	{
		return perceptors;
	}
}
