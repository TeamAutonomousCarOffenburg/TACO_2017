package taco.agent.decision.decisionmaker.impl;

import java.awt.Color;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.Polygon;
import taco.agent.communication.perception.JuryAction;
import taco.agent.communication.perception.RecognizedObjectType;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.IAudiCupMotor;

public class AudiCupDecisionMaker extends AudiCupDecisionMakerBase
{
	private final String driveBehavior;

	public AudiCupDecisionMaker(BehaviorMap behaviors, IThoughtModel thoughtModel, String driveBehavior)
	{
		super(behaviors, thoughtModel);
		this.driveBehavior = driveBehavior;
	}

	@Override
	public String decideNextBehavior()
	{
		if (getWorldModel().hasJuryActionChanged()) {
			behaviors.getMap().values().forEach(IBehavior::init);
			reset();
		}

		if (!getThoughtModel().isInitialized()) {
			return NONE;
		}

		if (getWorldModel().getJuryAction() != JuryAction.START) {
			return STOP;
		}

		return driveBehavior;
	}

	@Override
	protected void onBehaviorPerformed()
	{
		getThoughtModel().updateAfterPerform();

		getWorldModel().getLaneMiddleSensor().updateConsecutiveFollowRightLanePerforms(
				getBehavior(FOLLOW_RIGHT_LANE).getConsecutivePerforms());

		// adaptive cruise control checks if speed should be reduced due to obstacle
		adaptiveCruiseControl();

		if (!currentBehavior.getName().equals(IBehaviorConstants.DRIVE_SLALOM)) {
			slowDownForPedestrians();

			// check if we have to do an emergency brake due to obstacle
			brakeAssist();
		}
	}

	private void adaptiveCruiseControl()
	{
		if (getThoughtModel().isMovingObstacleAhead()) {
			double speed = getAgentModel().getMotor().getTargetSpeed();
			double distance = 1;
			double obstacleDistance = getThoughtModel().getObstacleAheadDistance(distance);

			double limitedSpeed = Geometry.getLinearFuzzyValue(0.3, 2.3, true, obstacleDistance) * 80;
			speed = Math.min(speed, limitedSpeed);
			if (speed < 21) {
				speed = 21;
			}
			getAgentModel().getMotor().drive(speed);
		}
	}

	private void slowDownForPedestrians()
	{
		// TODO: Adjust polygon
		Vector2D bottomLeft = new Vector2D(0.5, 1);
		Vector2D bottomRight = new Vector2D(0.5, -0.6);
		Vector2D topRight = new Vector2D(2.5, -1);
		Vector2D topLeft = new Vector2D(2.5, 1.4);
		Polygon viewRange = new Polygon(bottomLeft, bottomRight, topRight, topLeft)
									.transform(getWorldModel().getThisCar().getPose());

		boolean personCrossingOrChildNearby =
				getWorldModel()
						.getRecognizedObjects()
						.stream()
						.filter(o
								-> o.getType() == RecognizedObjectType.CHILD ||
										   o.getType() == RecognizedObjectType.PERSON_CROSSING)
						.anyMatch(pedestrian -> viewRange.intersects(pedestrian.getArea()));
		if (personCrossingOrChildNearby && getAgentModel().getMotor().getTargetSpeed() > IAudiCupMotor.LOW_SPEED) {
			getAgentModel().getMotor().drive(IAudiCupMotor.LOW_SPEED);
		}
		getThoughtModel().getDrawings().draw("pedestrianSlowDownArea",
				personCrossingOrChildNearby ? new Color(248, 0, 0, 20) : new Color(248, 248, 248, 20), viewRange);
	}

	private void brakeAssist()
	{
		if (getThoughtModel().isObstacleInPath() || getThoughtModel().isPedestrianAhead()) {
			if (!currentBehavior.getName().equals(EMERGENCY_BRAKE)) {
				currentBehavior = getBehavior(EMERGENCY_BRAKE);
				currentBehavior.perform();
			}
		}
	}
}
