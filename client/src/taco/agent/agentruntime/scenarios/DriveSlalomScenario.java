package taco.agent.agentruntime.scenarios;

import taco.agent.decision.behavior.IBehaviorConstants;

public class DriveSlalomScenario extends ScenarioBase
{
	@Override
	public int getStartSector()
	{
		return 2;
	}

	@Override
	public String getDriveBehavior()
	{
		return IBehaviorConstants.DRIVE_SLALOM;
	}
}
