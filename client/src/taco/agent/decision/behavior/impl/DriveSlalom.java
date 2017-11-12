package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import taco.agent.communication.perception.RecognizedObjectType;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupComplexBehavior;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.worldmodel.impl.Obstacle;

import java.awt.Color;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class DriveSlalom extends AudiCupComplexBehavior
{
	private enum Phase { DRIVE_TO, CROSS, TURN_AROUND }

	private enum CrossingDirection {
		LEFT,
		RIGHT;

		public CrossingDirection flip()
		{
			return this == LEFT ? RIGHT : LEFT;
		}
	}

	private enum DriveDirection {
		NORTH,
		SOUTH;

		public DriveDirection flip()
		{
			return this == NORTH ? SOUTH : NORTH;
		}
	}

	private Phase phase = Phase.DRIVE_TO;

	private CrossingDirection nextCrossingDirection = CrossingDirection.RIGHT;

	private DriveDirection driveDirection = DriveDirection.NORTH;

	private Pose2D nextCrossing;

	private Pose2D nextDriveTo;

	private Angle turnStartAngle;

	private Obstacle previousObstacle;

	private float lastValidTargetTime;

	public DriveSlalom(IThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(IBehaviorConstants.DRIVE_SLALOM, thoughtModel, behaviors);
	}

	@Override
	protected String decideNextBasicBehavior()
	{
		List<Obstacle> closestObstacles = getClosestObstacles();
		float time = getWorldModel().getGlobalTime();
		Angle carAngle = getWorldModel().getThisCar().getPose().getAngle();

		if (phase == Phase.TURN_AROUND) {
			if (closestObstacles.isEmpty() ||
					Math.abs(carAngle.degreesPositive() - turnStartAngle.degreesPositive()) < 150) {
				int steering = nextCrossingDirection == CrossingDirection.LEFT ? 30 : -30;
				if (driveDirection == DriveDirection.NORTH) {
					steering *= -1;
				}
				getAgentModel().getSteering().steer(Angle.deg(steering));
				getAgentModel().getMotor().drive(IAudiCupMotor.DEFAULT_SPEED);
				return NONE;
			} else {
				switchPhase();
			}
		}

		determineNextPoints(closestObstacles);
		Pose2D nextTarget = getNextTarget();

		// are we past the target? if so, switch to next phase
		if (phase != Phase.TURN_AROUND && isBehindUs(nextTarget)) {
			switchPhase();
			nextTarget = getNextTarget();
		}

		if (nextTarget != null) {
			lastValidTargetTime = time;
		} else {
			if (time - lastValidTargetTime > 1) {
				if (phase != Phase.TURN_AROUND) {
					phase = Phase.TURN_AROUND;
					turnStartAngle = carAngle;
					driveDirection = driveDirection.flip();
					nextCrossingDirection = nextCrossingDirection.flip();
				}
				return NONE;
			} else if (time - lastValidTargetTime > 1) {
				return IBehaviorConstants.STOP;
			} else {
				getAgentModel().getMotor().drive(IAudiCupMotor.LOW_SPEED);
				return IBehaviorConstants.STOP;
			}
		}

		getThoughtModel().getDrawings().draw("nextCrossing", Color.YELLOW, nextCrossing);
		getThoughtModel().getDrawings().draw("nextDriveTo", Color.YELLOW, nextDriveTo);
		getThoughtModel().getDrawings().draw("nextTarget", Color.RED, nextTarget);
		getThoughtModel().log("phase", phase + "," + nextCrossingDirection);

		DriveToPose driveToPose = (DriveToPose) behaviors.get(IBehaviorConstants.DRIVE_TO_POSE);
		driveToPose.setTargetPose(nextTarget, nextTarget, IAudiCupMotor.DEFAULT_SPEED);
		return IBehaviorConstants.DRIVE_TO_POSE;
	}

	private Pose2D getNextTarget()
	{
		switch (phase) {
		case DRIVE_TO:
			return nextDriveTo;

		case CROSS:
			return nextCrossing;
		}
		return null;
	}

	private boolean isBehindUs(Pose2D pose)
	{
		if (pose == null) {
			return false;
		}
		IPose2D carPose = getWorldModel().getThisCar().getPose();
		return carPose.applyInverseTo(pose).getX() < 0.3;
	}

	private void determineNextPoints(List<Obstacle> closestObstacles)
	{
		getThoughtModel().getDrawings().draw("relevantObstacles", Color.WHITE,
				closestObstacles.stream().map(Obstacle::getPosition).toArray(Vector3D[] ::new));

		if (closestObstacles.size() < 1) {
			return;
		}

		int crossingDirection = nextCrossingDirection == CrossingDirection.RIGHT ? -1 : 1;
		if (phase == Phase.CROSS) {
			crossingDirection *= -1;
		}

		Angle directionAngle = driveDirection == DriveDirection.NORTH ? Angle.ZERO : Angle.deg(180);

		Obstacle closestObstacle = closestObstacles.get(0);
		Vector3D closestObstaclePos = closestObstacle.getPosition();

		getThoughtModel().getDrawings().draw("closestObstaclePos", Color.RED, closestObstaclePos);

		Area2D.Float area = closestObstacle.getArea();
		float obstacleX = driveDirection == DriveDirection.SOUTH ? area.getMinX() : area.getMaxX();
		float obstacleY = crossingDirection == -1 ? area.getMinY() : area.getMaxY();
		Vector3D obstaclePos = new Vector3D(obstacleX, obstacleY, 0);

		Pose2D potentialDriveTo =
				new Pose2D(obstaclePos.add(crossingDirection, new Vector3D(0, 0.4, 0)), directionAngle);

		// allow corrections of up to 0.5 m
		if (nextDriveTo == null || potentialDriveTo.getDistanceTo(nextDriveTo) < 0.5) {
			nextDriveTo = potentialDriveTo;
			if (phase == Phase.DRIVE_TO &&
					(previousObstacle == null || previousObstacle.getPosition().distance(closestObstaclePos) > 1.0)) {
				previousObstacle = closestObstacle;
			}
		}

		if (phase == Phase.CROSS && previousObstacle != null) {
			Vector3D previousObstaclePos = previousObstacle.getPosition();
			if (driveDirection == DriveDirection.SOUTH) {
				crossingDirection *= -1;
			}
			nextCrossing = new Pose2D(previousObstaclePos.add(0.5, closestObstaclePos.subtract(previousObstaclePos)),
					Angle.deg(45 * crossingDirection).add(directionAngle));
		}
	}

	private List<Obstacle> getClosestObstacles()
	{
		IPose2D carPose = getWorldModel().getThisCar().getPose();
		return getWorldModel()
				.getRecognizedObjects()
				.stream()
				.filter(obstacle -> obstacle.getType().isPedestrian() || obstacle.getType() == RecognizedObjectType.CAR)
				.filter(obstacle -> carPose.applyInverseTo(obstacle.getPosition()).getX() > 0.3)
				.sorted(Comparator.comparingDouble(o -> o.getDistanceToXY(carPose.getPosition())))
				.collect(Collectors.toList());
	}

	private void switchPhase()
	{
		switch (phase) {
		case TURN_AROUND:
		case DRIVE_TO:
			phase = Phase.CROSS;
			nextDriveTo = null;
			break;

		case CROSS:
			phase = Phase.DRIVE_TO;
			nextCrossingDirection = nextCrossingDirection.flip();
			nextCrossing = null;
			break;
		}
	}
}
