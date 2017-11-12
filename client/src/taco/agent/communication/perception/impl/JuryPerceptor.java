package taco.agent.communication.perception.impl;

import taco.agent.communication.perception.IJuryPerceptor;
import taco.agent.communication.perception.JuryAction;
import taco.agent.communication.perception.PerceptorName;

public class JuryPerceptor extends AudiCupPerceptor implements IJuryPerceptor
{
	private int maneuverId;

	private JuryAction action;

	public JuryPerceptor(long timestamp, int maneuverId, JuryAction action)
	{
		super(PerceptorName.JURY_COMMAND, timestamp);
		this.maneuverId = maneuverId;
		this.action = action;
	}

	@Override
	public int getManeuverId()
	{
		return maneuverId;
	}

	@Override
	public JuryAction getAction()
	{
		return action;
	}
}
