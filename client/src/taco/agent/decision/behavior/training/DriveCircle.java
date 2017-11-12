package taco.agent.decision.behavior.training;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;

public class DriveCircle extends AudiCupBehavior
{
	private int circleCounter;

	private boolean halfCircle;

	public DriveCircle(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.DRIVE_CIRCLE, thoughtModel);
		circleCounter = 0;
		halfCircle = false;
	}

	@Override
	public void perform()
	{
		super.perform();

		double angle = getWorldModel().getThisCar().getPose().getAngle().degrees();
		if (angle > 90) {
			halfCircle = true;
		} else if (halfCircle && angle > -5 && angle < 5) {
			halfCircle = false;
			circleCounter++;
		}

		if (circleCounter < 3) {
			getAgentModel().getMotor().driveForward();
			getAgentModel().getSteering().steer(Angle.deg(-30));
		} else {
			getAgentModel().getMotor().stop();
		}
	}
}
