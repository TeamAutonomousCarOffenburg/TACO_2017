package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;
import taco.agent.model.agentmodel.impl.enums.TachometerPosition;

/**
 * Braking due to obstacles.
 * The duration to stay in this behavior depends on that this behavior is called as long as it is not finished
 */
public class EmergencyBrake extends AudiCupBehavior
{
	protected enum Phase { FINISHED, BRAKING, WAITING }

	private static final float BRAKE_DURATION = 1;

	/** how long to wait after braking (in s) */
	private static final float WAIT_DURATION = 2;

	private Phase phase;

	private double previousSpeed;

	/** the time we started braking */
	private float phaseStartTime;

	public EmergencyBrake(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.EMERGENCY_BRAKE, thoughtModel);
	}

	@Override
	public void init()
	{
		super.init();
		phase = Phase.FINISHED;
		previousSpeed = 0;
		phaseStartTime = 0;
	}

	@Override
	public boolean isFinished()
	{
		return phase == Phase.FINISHED;
	}

	@Override
	public void perform()
	{
		double targetSpeed = 0;

		float time = getWorldModel().getGlobalTime();
		switch (phase) {
		case FINISHED:
			// first call
			previousSpeed = getAgentModel().getMotor().getTargetSpeed();
			if (previousSpeed == 0) {
				switchPhase(Phase.WAITING);
			} else {
				switchPhase(Phase.BRAKING);
			}
			break;

		case BRAKING:
			double absKmh = Math.abs(getAgentModel().getTachometer(TachometerPosition.LEFT).getSpeed());
			if (time - phaseStartTime >= BRAKE_DURATION || absKmh < 1) {
				switchPhase(Phase.WAITING);
			} else {
				// drive in opposite direction to stand still more quickly
				targetSpeed = -previousSpeed;
			}
			break;

		case WAITING:
			if (time - phaseStartTime >= WAIT_DURATION) {
				switchPhase(Phase.FINISHED);
			}
			break;
		}

		getAgentModel().getMotor().drive(targetSpeed);
		super.perform();
	}

	private void switchPhase(Phase state)
	{
		this.phase = state;
		phaseStartTime = getWorldModel().getGlobalTime();
	}

	@Override
	public IBehavior switchFrom(IBehavior actualBehavior)
	{
		actualBehavior.onLeavingBehavior(this);
		return this;
	}
}
