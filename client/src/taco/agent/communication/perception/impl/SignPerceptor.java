package taco.agent.communication.perception.impl;

import java.util.List;

import taco.agent.communication.perception.ISignPerceptor;
import taco.agent.communication.perception.PerceptorName;
import taco.agent.model.worldmodel.signdetection.RoadSign;

public class SignPerceptor extends AudiCupPerceptor implements ISignPerceptor
{
	private List<RoadSign> signs;

	public SignPerceptor(long timestamp, List<RoadSign> signs)
	{
		super(PerceptorName.SIGNS, timestamp);
		this.signs = signs;
	}

	@Override
	public List<RoadSign> getSigns()
	{
		return signs;
	}
}
