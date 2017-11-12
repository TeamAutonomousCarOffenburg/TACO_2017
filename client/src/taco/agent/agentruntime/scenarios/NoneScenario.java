package taco.agent.agentruntime.scenarios;

import taco.agent.decision.behavior.IBehaviorConstants;

public class NoneScenario extends ScenarioBase
{
	@Override
	public String getDriveBehavior()
	{
		return IBehaviorConstants.NONE;
	}
}
