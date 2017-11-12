package taco.agent.agentruntime.scenarios;

import hso.autonomy.agent.decision.behavior.IBehavior;
import taco.agent.decision.behavior.training.DriveStraight;
import taco.agent.model.agentmodel.IAudiCupMotor;

public class StopAtHighSpeedScenario extends ScenarioBase
{
	@Override
	public String getDriveBehavior()
	{
		return DRIVE_STRAIGHT;
	}

	@Override
	public void configureDriveBehavior(IBehavior behavior)
	{
		DriveStraight driveStraight = (DriveStraight) behavior;
		driveStraight.setSpeed(IAudiCupMotor.HIGH_SPEED);
		driveStraight.setDistance(5);
	}

	@Override
	public int getStartSector()
	{
		return 2;
	}
}
