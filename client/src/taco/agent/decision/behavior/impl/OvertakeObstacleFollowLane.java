package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupComplexBehavior;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.agentmodel.IUltrasonic;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.agentmodel.impl.enums.UltrasonicPosition;

import java.util.ArrayList;
import java.util.List;

public class OvertakeObstacleFollowLane extends AudiCupComplexBehavior
{
	private enum Phase { DRIVE, BACKWARDS, FORWARD_LEFT, OVERTAKE, FORWARD_RIGHT, ENDING, FINISHED }

	private Phase phase;

	private List<Double> distancesSide;

	private IPose2D overtakenPosition;
	private IPose2D startPosition;
	private boolean isMoving, initCheck;

	private FollowLeftLane followLeftLane;

	public OvertakeObstacleFollowLane(IThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(IBehaviorConstants.OVERTAKE_OBSTACLE, thoughtModel, behaviors);

		followLeftLane = (FollowLeftLane) behaviors.get(IBehaviorConstants.FOLLOW_LEFT_LANE);
	}

	@Override
	public void init()
	{
		super.init();
		phase = Phase.BACKWARDS;
		distancesSide = new ArrayList<>();
		overtakenPosition = null;
		startPosition = null;
		isMoving = false;
		initCheck = false;
	}

	@Override
	protected String decideNextBasicBehavior()
	{
		IAudiCupAgentModel agentModel = getAgentModel();
		IPose2D currentCarPosition = getWorldModel().getThisCar().getPose();

		checkInit();
		measureUSDistances();

		switch (phase) {
		case BACKWARDS:
			agentModel.getMotor().driveBackward();
			// agentModel.getSteering().reset();

			if (currentCarPosition.getDistanceTo(startPosition) > 0.8) {
				agentModel.getMotor().stop();
				phase = Phase.FORWARD_LEFT;
			}
			break;
		case FORWARD_LEFT:
			agentModel.getLight(LightName.INDICATOR_LEFT).turnOn();

			if (isMoving) {
				followLeftLane.setSpeed(IAudiCupMotor.HIGH_SPEED);
			}

			if (onSide()) {
				phase = Phase.OVERTAKE;
				agentModel.getLight(LightName.INDICATOR_LEFT).turnOff();
			}

			if (checkAbort(currentCarPosition)) {
				getThoughtModel().log("overtakeAbort", true);
				phase = Phase.FORWARD_RIGHT;
			}

			return IBehaviorConstants.FOLLOW_LEFT_LANE;

		case OVERTAKE:
			// check for Obstacle in front of the vehicle, wee need this because we are use this behavior to overtake
			// an object
			IUltrasonic usFront = agentModel.getUltrasonic(UltrasonicPosition.FRONT_CENTER);
			if (usFront.getDistance() < 0.5) {
				agentModel.getMotor().stop();
				phase = Phase.FINISHED;
				return NONE;
			}

			if (overtakenPosition == null && !onSide()) {
				agentModel.getLight(LightName.INDICATOR_RIGHT).turnOn();
				overtakenPosition = currentCarPosition;
				phase = Phase.FORWARD_RIGHT;
			}

			if (checkAbort(currentCarPosition)) {
				getThoughtModel().log("overtakeAbort", true);
				phase = Phase.FORWARD_RIGHT;
			}

			return IBehaviorConstants.FOLLOW_LEFT_LANE;

		case FORWARD_RIGHT:
			if (overtakenPosition == null || currentCarPosition.getDistanceTo(overtakenPosition) > 0.3) {
				agentModel.getLight(LightName.INDICATOR_RIGHT).turnOff();
				if (isMoving) {
					followLeftLane.setSpeed(IAudiCupMotor.DEFAULT_SPEED);
				}
				phase = Phase.FINISHED;

				distancesSide.clear();
			}
			return IBehaviorConstants.FOLLOW_RIGHT_LANE;
		}

		return NONE;
	}

	private void checkInit()
	{
		if (!initCheck) {
			if (getThoughtModel().isMovingObstacleAhead()) {
				isMoving = true;
				phase = Phase.FORWARD_LEFT;
			}

			getThoughtModel().log("overtakeAbort", false);
			initCheck = true;
		}

		if (startPosition == null) {
			startPosition = getWorldModel().getThisCar().getPose();
		}
	}

	private void measureUSDistances()
	{
		IUltrasonic usSideRight = getAgentModel().getUltrasonic(UltrasonicPosition.SIDE_RIGHT);

		distancesSide.add(usSideRight.getDistance());

		if (distancesSide.size() > 40) {
			// keep history limited
			distancesSide.remove(0);
		}
	}

	private boolean onSide()
	{
		if (distancesSide.size() > 10 &&
				distancesSide.stream().filter(distance -> distance < 0.6 && distance > 0).count() > 5) {
			return true;
		}

		return false;
	}

	private boolean checkAbort(IPose2D currentCarPosition)
	{
		return (startPosition.getDistanceTo(currentCarPosition) > 3 && !onSide());
	}

	public boolean isFinished()
	{
		return phase == Phase.FINISHED;
	}
}