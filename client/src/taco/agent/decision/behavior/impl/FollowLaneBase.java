package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import taco.agent.decision.behavior.IFollowLane;
import taco.agent.decision.behavior.base.AudiCupBehavior;
import taco.agent.model.agentmeta.impl.ServoDriveConfiguration;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.util.drive.AngleUtils;

public abstract class FollowLaneBase extends AudiCupBehavior implements IFollowLane
{
	private float lastSteeringOut;

	protected double speed;

	protected float inputFactor;

	public FollowLaneBase(String name, IThoughtModel thoughtModel, FollowLaneParameters params)
	{
		super(name, thoughtModel);
		if (params != null) {
			inputFactor = params.getInputFactor();
		} else {
			inputFactor = 0.13f;
		}
	}

	@Override
	public void setSpeed(double speed)
	{
		this.speed = speed;
	}

	@Override
	public void init()
	{
		super.init();
		lastSteeringOut = 0f;
		speed = IAudiCupMotor.DEFAULT_SPEED;
	}

	protected abstract int getDeltaX();

	@Override
	public void perform()
	{
		Angle steeringAngle = calculateSteeringAngle(getDeltaX());
		getAgentModel().getSteering().steer(steeringAngle);
		getAgentModel().getMotor().drive(speed);
	}

	/**
	 * Calculates the angle for steering.
	 * Copied from the C++ code.
	 *
	 *********************************
	 *
	 * PT 1 discrete algorithm
	 *
	 *               Tau
	 *       In +    ---  * LastOut
	 *             Tsample
	 * Out = ---------------------
	 *               Tau
	 *       1 +     ---
	 *             Tsample
	 *
	 *                           Tau
	 * here with     Factor =    ---
	 *                         Tsample
	 *
	 *******************************************
	 */
	private Angle calculateSteeringAngle(int deltaX)
	{
		float m_f64PT1Tau = 0.1f;
		float m_f64PT1Sample = 0.9f;
		float m_f64PT1Gain = m_f64PT1Tau / m_f64PT1Sample;
		float pT1ScaledError = deltaX * inputFactor;

		float pT1SteeringOut = (pT1ScaledError + m_f64PT1Gain * lastSteeringOut) / (1 + m_f64PT1Gain);
		lastSteeringOut = pT1SteeringOut;

		ServoDriveConfiguration config = getAgentModel().getCarMetaModel().getServoDriveConfigs()[0];
		Angle steeringAngle = Angle.deg(-pT1SteeringOut);
		return AngleUtils.limit(steeringAngle, config.getMin(), config.getMax());
	}
}
