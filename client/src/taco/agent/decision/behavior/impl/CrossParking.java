package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Polygon;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.IFollowLane;
import taco.agent.decision.behavior.base.AudiCupComplexBehavior;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.worldmodel.impl.ParkingSpace;

import java.awt.*;

public class CrossParking extends AudiCupComplexBehavior
{
	private enum Phase { DRIVE_TO_SPACE, FORWARD_LEFT, BACKWARD_RIGHT, BACKWARD, ENDING, FINISHED }

	private final IFollowLane followLane;

	private final DriveToPose driveToPose;

	private Phase phase;

	private ParkingSpace targetParkingSpace;

	private Polygon spaceMeasurementArea;

	private int desiredParkingSpace;

	private IPose2D startPosition;

	private IPose2D next;
	private IPose2D afterNext;

	public CrossParking(IThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(IBehaviorConstants.CROSS_PARKING, thoughtModel, behaviors);
		this.followLane = (IFollowLane) behaviors.get(IBehaviorConstants.FOLLOW_RIGHT_LANE);
		this.driveToPose = (DriveToPose) behaviors.get(IBehaviorConstants.DRIVE_TO_POSE);
	}

	public void setDesiredParkingSpace(int id)
	{
		desiredParkingSpace = id;
	}

	@Override
	public boolean isFinished()
	{
		return phase == Phase.FINISHED;
	}

	@Override
	public void init()
	{
		super.init();
		phase = Phase.DRIVE_TO_SPACE;
		targetParkingSpace = null;
		spaceMeasurementArea = null;
		desiredParkingSpace = 2;
		next = null;
		afterNext = null;
	}

	@Override
	protected String decideNextBasicBehavior()
	{
		if (targetParkingSpace == null) {
			targetParkingSpace = getWorldModel().getEnvironmentManager().getParkingSpaceById(desiredParkingSpace);
			if (targetParkingSpace == null) {
				return NONE;
			}

			if (startPosition == null) {
				startPosition = getWorldModel().getThisCar().getPose();
			}
		}

		IPose2D currentCarPosition = getWorldModel().getThisCar().getPose();
		IPose2D targetSpacePosition = targetParkingSpace.getPose();
		float time = getWorldModel().getGlobalTime();
		IAudiCupAgentModel agentModel = getAgentModel();

		getThoughtModel().log("parkingPhase", phase);

		switch (phase) {
		case DRIVE_TO_SPACE:
			agentModel.getLight(LightName.INDICATOR_RIGHT).turnOn();
			followLane.setSpeed(IAudiCupMotor.LOW_SPEED);

			if (next == null) {
				next = targetSpacePosition.applyTo(new Pose2D(0, 0.25));
				afterNext = targetSpacePosition.applyTo(new Pose2D(0.5, 0.5, Angle.deg(45)));
				drawNextPoints(next, afterNext);
				driveToPose.setTargetPose(next, afterNext, IAudiCupMotor.LOW_SPEED, true);
			}

			if (currentCarPosition.getDistanceTo(next) < 0.1) {
				phase = Phase.FORWARD_LEFT;
				next = null;
			}

			return IBehaviorConstants.DRIVE_TO_POSE;
		case FORWARD_LEFT:
			if (next == null) {
				next = afterNext;
				afterNext = next.applyTo(new Pose2D(1, 0));
				drawNextPoints(next, afterNext);
				driveToPose.setTargetPose(next, afterNext, IAudiCupMotor.LOW_SPEED, true);
			}

			if (currentCarPosition.getDistanceTo(next) < 0.1) {
				phase = Phase.BACKWARD_RIGHT;
				next = null;
			}

			return IBehaviorConstants.DRIVE_TO_POSE;

		case BACKWARD_RIGHT:
			if (next == null) {
				next = targetSpacePosition.applyTo(new Pose2D(0, 0, Angle.ANGLE_90));
				afterNext = next.applyTo(new Pose2D(-0.7, 0));
				drawNextPoints(next, afterNext);
				driveToPose.setTargetPose(next, afterNext, IAudiCupMotor.LOW_SPEED, true);
			}

			if (currentCarPosition.getDistanceTo(next) < 0.1) {
				phase = Phase.BACKWARD;
				next = null;
				agentModel.getLight(LightName.INDICATOR_RIGHT).turnOff();
			}

			return IBehaviorConstants.DRIVE_TO_POSE;
		case BACKWARD:
			if (next == null) {
				next = afterNext;
				afterNext = next.applyTo(new Pose2D(-3, 0));
				drawNextPoints(next, afterNext);
				driveToPose.setTargetPose(next, afterNext, IAudiCupMotor.LOW_SPEED, true);
			}

			if (currentCarPosition.getDistanceTo(next) < 0.1) {
				phase = Phase.ENDING;
				next = null;
				agentModel.getMotor().stop();
				agentModel.getLight(LightName.WARN).turnOn();

				return NONE;
			}

			return IBehaviorConstants.DRIVE_TO_POSE;
		case ENDING:
			if (time - agentModel.getLight(LightName.WARN).getModificationTime() > 4f) {
				agentModel.getLight(LightName.WARN).turnOff();
				phase = Phase.FINISHED;
			}
			break;

		case FINISHED:
		}

		return NONE;
	}

	private void drawNextPoints(IPose2D next, IPose2D afterNext)
	{
		getThoughtModel().getDrawings().draw("next", new Color(161, 40, 255, 128), next);
		getThoughtModel().getDrawings().draw("afterNext", new Color(255, 196, 0, 128), afterNext);
		getThoughtModel().log("next", next);
		getThoughtModel().log("afterNext", afterNext);
	}
}
