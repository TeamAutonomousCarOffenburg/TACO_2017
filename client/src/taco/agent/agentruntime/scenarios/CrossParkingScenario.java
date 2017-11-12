package taco.agent.agentruntime.scenarios;

import hso.autonomy.agent.decision.behavior.IBehavior;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.impl.CrossParking;

public class CrossParkingScenario extends ScenarioBase
{
	@Override
	public int getStartSector()
	{
		return 1;
	}

	@Override
	public String getDriveBehavior()
	{
		return IBehaviorConstants.CROSS_PARKING;
	}

	@Override
	public void configureDriveBehavior(IBehavior behavior)
	{
		CrossParking crossParking = (CrossParking) behavior;
		crossParking.setDesiredParkingSpace(2);
	}
}
