package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;

public class Stop extends AudiCupBehavior
{
	public Stop(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.STOP, thoughtModel);
	}

	@Override
	public void perform()
	{
		getAgentModel().getMotor().stop();
	}

	@Override
	public IBehavior switchFrom(IBehavior actualBehavior)
	{
		actualBehavior.onLeavingBehavior(this);
		return this;
	}
}
