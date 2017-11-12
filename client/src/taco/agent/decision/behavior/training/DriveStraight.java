package taco.agent.decision.behavior.training;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupComplexBehavior;
import taco.agent.model.agentmodel.IAudiCupMotor;

/**
 * Behavior for demonstration purposes. Goal of Milestone 1.
 */
public class DriveStraight extends AudiCupComplexBehavior
{
	private double distance = 1;

	private double speed = IAudiCupMotor.DEFAULT_SPEED;

	public DriveStraight(IThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(IBehaviorConstants.DRIVE_STRAIGHT, thoughtModel, behaviors);
	}

	@Override
	protected String decideNextBasicBehavior()
	{
		getAgentModel().getSteering().reset();
		IAudiCupMotor motor = getAgentModel().getMotor();

		if (getWorldModel().getGyroOdometry().getDrivenDistance() < distance) {
			motor.drive(speed);
			return NONE;
		} else {
			return IBehaviorConstants.EMERGENCY_BRAKE;
		}
	}

	public void setDistance(double distance)
	{
		this.distance = distance;
	}

	public void setSpeed(double speed)
	{
		this.speed = speed;
	}
}
