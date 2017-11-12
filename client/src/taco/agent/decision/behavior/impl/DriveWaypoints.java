package taco.agent.decision.behavior.impl;

import java.util.ArrayList;
import java.util.List;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.IFollowLane;
import taco.agent.decision.behavior.base.AudiCupComplexBehavior;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.impl.DrivePoint;
import taco.agent.model.worldmodel.impl.Maneuver;
import taco.agent.model.worldmodel.lanedetection.LaneMiddle;
import taco.agent.model.worldmodel.lanedetection.LaneMiddleSensor;
import taco.agent.model.worldmodel.street.*;

public class DriveWaypoints extends AudiCupComplexBehavior
{
	private static final boolean FOLLOW_LANE_ENABLED = true;

	private static final boolean DRIVE_GEOMETRY_ENABLED = true;

	private static final double INDICATOR_DISTANCE = 0.9;

	private static final double GIVE_WAY_DISTANCE = 0.8;

	private static final double TURN_RIGHT_DISTANCE = 0.5;

	private static final double TURN_LEFT_DISTANCE = 0.15;

	private enum DriveState {
		PROCESSING_INSTRUCTION,
		DRIVING,
		APPROACHING_CROSSING,
		GIVING_WAY,
		PARKING,
		PULLING_OUT_RIGHT,
		PULLING_OUT_LEFT,
		OVERTAKE_OBSTACLE
	}

	/** the current state this behavior is in */
	private DriveState state;

	private boolean initialized;

	private boolean finished;

	/** id of the segment we happened to be in last cycle */
	private RuntimeSegment previousSegment;

	public DriveWaypoints(IAudiCupThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(IBehaviorConstants.DRIVE_WAYPOINTS, thoughtModel, behaviors);
	}

	@Override
	public void init()
	{
		super.init();
		state = DriveState.PROCESSING_INSTRUCTION;
		initialized = false;
		finished = false;
	}

	@Override
	protected String decideNextBasicBehavior()
	{
		checkStateSwitch();
		checkDrivePathUpdate();

		List<DrivePoint> path = new ArrayList<>(getWorldModel().getThisCar().getPath().getDrivePath());
		if (reachedEndOfPath(path)) {
			return NONE;
		}

		DrivePoint nextPoint = path.get(1);
		double distanceToNextPoint = getWorldModel().getThisCar().getPose().getDistanceTo(nextPoint.getPose());

		// only finish up driving to last point?
		if (path.size() < 3) {
			// we add a dummy drive instruction simply not to have to treat this as special case
			path.add(new DrivePoint(nextPoint.getGoalLink(), new Pose2D(),
					new Maneuver(DriveInstruction.FOLLOW_LANE, 0, nextPoint.getInstructionIndex() + 1)));
		}

		// we have at least one more maneuver
		String behavior = executeNextInstruction(path, distanceToNextPoint);
		getThoughtModel().log("state", state);
		return behavior;
	}

	private void checkStateSwitch()
	{
		if (!getCurrentBehavior().isFinished()) {
			return;
		}

		switch (getCurrentBehavior().getName()) {
		case IBehaviorConstants.GIVE_WAY:
			state = DriveState.DRIVING;
			break;

		case IBehaviorConstants.CROSS_PARKING:
		case IBehaviorConstants.PULL_OUT_LEFT:
		case IBehaviorConstants.PULL_OUT_RIGHT:
		case IBehaviorConstants.OVERTAKE_OBSTACLE:
			state = DriveState.PROCESSING_INSTRUCTION;
			break;
		}
	}

	private boolean reachedEndOfPath(List<DrivePoint> path)
	{
		int currentInstructionIndex = getWorldModel().getDriveInstructionManager().getCurrentInstructionIndex();
		int numberOfInstructions = getWorldModel().getDriveInstructionManager().getNumberOfInstructions();

		if (path.size() > 2) {
			finished = false;
		}

		boolean endReached = path.size() < 2 || currentInstructionIndex >= numberOfInstructions;
		if (!endReached && path.size() == 2) {
			// driving the last trajectory
			DrivePoint nextPoint = path.get(1);
			double distanceToNextPoint = getWorldModel().getThisCar().getPose().getDistanceTo(nextPoint.getPose());
			if (distanceToNextPoint < 0.05) {
				// we are close enough to last pose to stop
				endReached = true;
			}
		}

		if (endReached) {
			// end of path reached
			if (finished) {
				getAgentModel().getMotor().stop();
				// indicate end of sector
				turnOffIndicators();
				getAgentModel().getLight(LightName.WARN).turnOn();

			} else {
				// finish current sub behavior then stop
				finished = getCurrentBehavior().isFinished();
			}
			return true;
		}
		return false;
	}

	private void checkDrivePathUpdate()
	{
		RuntimeSegment currentSegment = getWorldModel().getCurrentSegment();
		DriveInstructionManager manager = getWorldModel().getDriveInstructionManager();

		// lazy initialization of drive path
		if (!initialized) {
			updateDrivePath();
			initialized = true;
		}

		if (currentSegment.getID() == previousSegment.getID()) {
			// no change in segment
			return;
		}

		// check with the still old intended option
		Segment nextSegment = previousSegment.getIntendedOption().getSegmentAfter();
		if (nextSegment == null || currentSegment.getID() != nextSegment.getID()) {
			if (manager.getCurrentInstruction() == DriveInstruction.CROSS_PARKING) {
				// normal in cross parking
				return;
			}
			// we did not change to the expected segment. What should we do?
			System.out.println("Did not change to expected segment");
		}

		// Progress driving instruction if necessary
		if (previousSegment.consumesDriveInstruction()) {
			manager.progressInstructionIndex();
			getAgentModel().getDriveStatus().setManeuverId(manager.getCurrentInstructionIndex());
		}

		updateDrivePath();
	}

	private void updateDrivePath()
	{
		RuntimeSegment currentSegment = getWorldModel().getCurrentSegment();
		DriveInstructionManager manager = getWorldModel().getDriveInstructionManager();

		currentSegment.update(manager.getPreviousManeuver(), manager.getCurrentManeuver());

		// Extract new virtual path
		getWorldModel().updateDrivePath();
		// it is necessary to keep a copy since the original will change
		previousSegment = new RuntimeSegment(currentSegment);
	}

	private String executeNextInstruction(List<DrivePoint> path, double distanceToNextPoint)
	{
		DrivePoint previousPoint = path.get(0);
		DrivePoint nextPoint = path.get(1);
		DrivePoint pointAfterNext = path.get(2);

		DriveInstruction instruction = nextPoint.getInstruction();
		DriveInstruction nextInstruction = pointAfterNext.getInstruction();

		if (checkOvertakeObstacle()) {
			if (!getCurrentBehavior().getName().equals(IBehaviorConstants.OVERTAKE_OBSTACLE)) {
				getCurrentBehavior().abort();
			}

			state = DriveState.OVERTAKE_OBSTACLE;
		}

		switch (state) {
		case PROCESSING_INSTRUCTION:
			processInstruction(distanceToNextPoint, instruction, nextInstruction);
			return drive(previousPoint, nextPoint, pointAfterNext);

		case DRIVING:
			if (switchToNextPose(distanceToNextPoint, instruction, nextInstruction)) {
				previousPoint = nextPoint;
				nextPoint = pointAfterNext;
			}
			return drive(previousPoint, nextPoint, pointAfterNext);

		case APPROACHING_CROSSING:
			if (distanceToNextPoint < GIVE_WAY_DISTANCE) {
				state = DriveState.GIVING_WAY;
				return NONE;
			} else {
				return drive(previousPoint, nextPoint, pointAfterNext);
			}

		default:
			return getSubBehavior(nextPoint);
		}
	}

	private String getSubBehavior(DrivePoint nextPoint)
	{
		switch (state) {
		case GIVING_WAY:
			return IBehaviorConstants.GIVE_WAY;

		case PARKING:
			((CrossParking) behaviors.get(IBehaviorConstants.CROSS_PARKING))
					.setDesiredParkingSpace(nextPoint.getManeuver().getInstructionSubID());
			return IBehaviorConstants.CROSS_PARKING;

		case PULLING_OUT_RIGHT:
			return IBehaviorConstants.PULL_OUT_RIGHT;

		case PULLING_OUT_LEFT:
			return IBehaviorConstants.PULL_OUT_LEFT;

		case OVERTAKE_OBSTACLE:
			return IBehaviorConstants.OVERTAKE_OBSTACLE;

		default:
			return NONE;
		}
	}

	private boolean switchToNextPose(
			double distanceToNextPoint, DriveInstruction instruction, DriveInstruction nextInstruction)
	{
		boolean isTurning = instruction == DriveInstruction.LEFT || instruction == DriveInstruction.RIGHT;
		if (isTurning || distanceToNextPoint < INDICATOR_DISTANCE) {
			double turnDistance = 0.0;
			if (nextInstruction == DriveInstruction.LEFT) {
				turnDistance = TURN_LEFT_DISTANCE;
			} else if (nextInstruction == DriveInstruction.RIGHT) {
				turnDistance = TURN_RIGHT_DISTANCE;
			}

			// close to the current destination we switch to reach next pose
			if (distanceToNextPoint < turnDistance) {
				return true;
			}
		} else {
			state = DriveState.PROCESSING_INSTRUCTION;
			turnOffIndicators();
		}
		return false;
	}

	private void processInstruction(
			double distanceToNextPoint, DriveInstruction currentInstruction, DriveInstruction nextInstruction)
	{
		switch (currentInstruction) {
		case CROSS_PARKING:
			state = DriveState.PARKING;
			break;

		case PULL_OUT_RIGHT:
			state = DriveState.PULLING_OUT_RIGHT;
			break;

		case PULL_OUT_LEFT:
			state = DriveState.PULLING_OUT_LEFT;
			break;

		default:
			if (distanceToNextPoint < INDICATOR_DISTANCE) {
				switch (nextInstruction) {
				case STRAIGHT:
				case STRAIGHT_FOREVER:
					state = DriveState.APPROACHING_CROSSING;
					break;

				case LEFT:
					state = DriveState.APPROACHING_CROSSING;
					getAgentModel().getLight(LightName.INDICATOR_LEFT).turnOn();
					break;

				case RIGHT:
					state = DriveState.APPROACHING_CROSSING;
					getAgentModel().getLight(LightName.INDICATOR_RIGHT).turnOn();
					break;

				default:
					// only change state if we approach a crossing
					break;
				}
			}
		}
	}

	@SuppressWarnings("UnusedAssignment")
	private String drive(DrivePoint previousPoint, DrivePoint nextPoint, DrivePoint pointAfterNext)
	{
		Angle angle = Angle.ZERO;
		double speed = nextPoint.getSpeed();

		switch (pointAfterNext.getInstruction()) {
		case STRAIGHT:
		case LEFT:
		case RIGHT:
			speed = IAudiCupMotor.LOW_SPEED;
		}

		double reliabilityThreshold = LaneMiddleSensor.DEFAULT_RELIABILITY_THRESHOLD;
		IPose2D scanLinePose =
				getWorldModel().getThisCar().getPose().applyTo(new Pose2D(LaneMiddle.SCANLINE_DISTANCE, 0));
		Segment scanLineSegment = getWorldModel().getMap().getSegmentContaining(scanLinePose.getPosition());
		if (scanLineSegment == null) {
			scanLineSegment = getWorldModel().getCurrentSegment().getSegment();
		}
		if (scanLineSegment.isSCurve() || scanLineSegment.isCurve()) {
			// always confident
			reliabilityThreshold = 0;
		}

		switch (nextPoint.getInstruction()) {
		case FOLLOW_LANE:
		case STRAIGHT:
		case STRAIGHT_FOREVER:
			boolean laneMiddleReliable = getWorldModel().getLaneMiddleSensor().isReliable(reliabilityThreshold);
			if (FOLLOW_LANE_ENABLED && laneMiddleReliable && !willTurn(previousPoint, nextPoint)) {
				IFollowLane followLane = (IFollowLane) behaviors.get(IBehaviorConstants.FOLLOW_RIGHT_LANE);
				followLane.setSpeed(speed);
				return IBehaviorConstants.FOLLOW_RIGHT_LANE;
			} else {
				int amount = 15;
				if (willTurn(previousPoint, nextPoint)) {
					amount = 30;
				}
				Direction previousDirection = previousPoint.getGoalLink().getDirection();
				Direction nextDirection = nextPoint.getGoalLink().getDirection();
				if (Direction.getLeft(previousDirection) == nextDirection) {
					angle = Angle.deg(amount); // left curve
				} else if (Direction.getRight(previousDirection) == nextDirection) {
					angle = Angle.deg(-amount); // right curve
				} else {
					angle = Angle.ZERO;
				}
			}
			break;

		case LEFT:
			angle = Angle.deg(30);
			break;

		case RIGHT:
			angle = Angle.deg(-30);
			break;
		}

		if (DRIVE_GEOMETRY_ENABLED) {
			DriveToPose driveToPose = (DriveToPose) behaviors.get(IBehaviorConstants.DRIVE_TO_POSE);
			driveToPose.setTargetPose(nextPoint.getPose(), pointAfterNext.getPose(), speed);
			return IBehaviorConstants.DRIVE_TO_POSE;
		} else {
			getAgentModel().getSteering().steer(angle);
			getAgentModel().getMotor().driveForward();
			return IBehaviorConstants.NONE;
		}
	}

	private boolean willTurn(DrivePoint previousPoint, DrivePoint nextPoint)
	{
		if (isTurningPoint(previousPoint)) {
			// in crossing
			return true;
		}
		if (isTurningPoint(nextPoint) &&
				nextPoint.getGoalLink().getPose().getDistanceTo(getWorldModel().getThisCar().getPose()) < 0.9) {
			// near crossing
			return true;
		}

		return false;
	}

	protected boolean isTurningPoint(DrivePoint point)
	{
		DriveInstruction instruction = point.getManeuver().getDriveInstruction();
		return instruction == DriveInstruction.RIGHT || instruction == DriveInstruction.LEFT;
	}

	private void turnOffIndicators()
	{
		getAgentModel().getLight(LightName.INDICATOR_LEFT).turnOff();
		getAgentModel().getLight(LightName.INDICATOR_RIGHT).turnOff();
	}

	private boolean checkOvertakeObstacle()
	{
		if (getThoughtModel().isPedestrianAhead()) {
			return false;
		}

		if (getWorldModel().closeToCrosswalk()) {
			return false;
		}

		if (getWorldModel().closeToCrossing()) {
			return false;
		}

		if (checkMovingObstacle() || checkFixObstacle()) {
			return true;
		}

		return false;
	}

	private boolean checkMovingObstacle()
	{
		return getThoughtModel().isMovingObstacleAhead() && getThoughtModel().followingObstacleSince() > 2f &&
				getWorldModel().isStraightStreet(4);
	}

	private boolean checkFixObstacle()
	{
		return (getThoughtModel().isObstacleAhead() && getAgentModel().getMotor().getTargetSpeed() == 0);
	}
}
