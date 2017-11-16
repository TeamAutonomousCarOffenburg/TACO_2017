package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.Polygon;
import taco.agent.communication.perception.RecognizedObjectType;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.impl.DrivePoint;
import taco.agent.model.worldmodel.impl.Obstacle;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.agent.model.worldmodel.street.Direction;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;
import taco.util.SignType;
import taco.util.rightofway.CarPositions;
import taco.util.rightofway.CrossingType;
import taco.util.rightofway.DriveDestination;
import taco.util.rightofway.IRightOfWayAnalyzer;
import taco.util.rightofway.RightOfWayAction;
import taco.util.rightofway.RightOfWayAnalyzer;

import java.awt.Color;
import java.util.List;

/** Behavior that waits until we have right of way */
public class GiveWay extends AudiCupBehavior
{
	/** the time we wait at a crossing before checking again if we have to give way (in s) */
	private static final float GIVE_WAY_WAIT_TIME = 3.0f;

	private static final float CROSSWALK_WAIT_TIME = 10.0f;

	private static final Polygon CROSSING_OPTION_AREA = new Polygon(new Area2D.Float(-0.7, 0.1, -0.25, 0.25));

	private enum Phase { CHECKING_ACTION, WAITING, FINISHED }

	private final IRightOfWayAnalyzer rightOfWayAnalyzer = new RightOfWayAnalyzer();

	private Phase phase;

	/** the time we started waiting before checking decision again */
	private float waitStartTime;

	/** true if we have stopped already at a stop and wait crossing */
	private boolean haveStopped;

	private float waitDuration;

	private Phase phaseAfterWait;

	public GiveWay(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.GIVE_WAY, thoughtModel);
	}

	@Override
	public void init()
	{
		super.init();
		phase = Phase.CHECKING_ACTION;
		waitStartTime = 0;
		haveStopped = false;
		waitDuration = 0;
		phaseAfterWait = null;
	}

	@Override
	public boolean isFinished()
	{
		return phase == Phase.FINISHED;
	}

	@Override
	public void perform()
	{
		getAgentModel().getSteering().reset();

		switch (phase) {
		case CHECKING_ACTION:
			List<DrivePoint> path = getWorldModel().getThisCar().getPath().getDrivePath();
			DrivePoint inPoint = path.get(1);

			if (getWorldModel().isCloseToCrosswalk()) {
				if (isPedestrianNearCrosswalk()) {
					stopAndWait(CROSSWALK_WAIT_TIME, Phase.FINISHED);
				}
			} else {
				RightOfWayAction action = determineRightOfWayAction(inPoint, path.get(2));

				if (action == RightOfWayAction.DRIVE) {
					phase = Phase.FINISHED;
				} else if (action == RightOfWayAction.STOP_THEN_DRIVE && haveStopped) {
					phase = Phase.FINISHED;
				} else {
					stopAndWait(GIVE_WAY_WAIT_TIME, Phase.CHECKING_ACTION);
				}
			}
			break;

		case WAITING:
			if (getWorldModel().getGlobalTime() - waitStartTime >= waitDuration) {
				// we check again next cycle
				phase = phaseAfterWait;
			}
			break;
		}
	}

	private boolean isPedestrianNearCrosswalk()
	{
		Segment crosswalk = getWorldModel().getCurrentSegment().getIntendedOption().getSegmentAfter();

		float borderX = -0.5f;
		float borderY = -0.6f;
		if (crosswalk.getRotation().equals(Angle.ANGLE_90)) {
			float temp = borderY;
			borderY = borderX;
			borderX = temp;
		}

		Area2D.Float area = crosswalk.getArea().applyBorder(borderX, borderY);

		boolean pedestrianNearCrosswalk = getWorldModel()
												  .getRecognizedObjects()
												  .stream()
												  .filter(o -> o.getType().isPedestrian())
												  .anyMatch(o -> area.contains(o.getPosition()));
		drawCheckArea("crosswalk", new Polygon(area), pedestrianNearCrosswalk);
		return pedestrianNearCrosswalk;
	}

	private void stopAndWait(float waitDuration, Phase phaseAfterWait)
	{
		this.waitDuration = waitDuration;
		this.phaseAfterWait = phaseAfterWait;

		getAgentModel().getMotor().stop();
		waitStartTime = getWorldModel().getGlobalTime();
		haveStopped = true;
		phase = Phase.WAITING;
	}

	private RightOfWayAction determineRightOfWayAction(DrivePoint inPoint, DrivePoint outPoint)
	{
		SegmentLink inLink = inPoint.getGoalLink();

		CrossingType type = inLink.getSegmentAfter().getCrossingType(inLink.getDirection());

		SignType signType = SignType.UNKNOWN;
		List<RoadSign> roadSigns = inLink.getRoadSigns();
		if (!roadSigns.isEmpty()) {
			signType = roadSigns.get(0).getSignType();
		}
		CarPositions carPositions = analyzeCarPositions(inPoint);
		DriveDestination destination = DriveDestination.NORTH;
		if (outPoint.getInstruction() == DriveInstruction.LEFT) {
			destination = DriveDestination.WEST;
		} else if (outPoint.getInstruction() == DriveInstruction.RIGHT) {
			destination = DriveDestination.EAST;
		}

		if ((type != CrossingType.NORTH_SOUTH_WEST || destination != DriveDestination.NORTH ||
					signType == SignType.STOP) &&
				isObstacleInCrossing(inLink)) {
			return RightOfWayAction.WAIT;
		}

		return rightOfWayAnalyzer.analyzeSituation(type, signType, carPositions, destination).action;
	}

	private boolean isObstacleInCrossing(SegmentLink inLink)
	{
		Segment crossingSegment = inLink.getSegmentAfter();
		Polygon crossingArea = new Polygon(crossingSegment.getArea().applyBorder(0.17f));

		boolean obstacleInCrossing = getWorldModel()
											 .getRecognizedObjects(RecognizedObjectType.CAR)
											 .stream()
											 .anyMatch(car -> crossingArea.intersects(car.getArea()));
		drawCheckArea("obstacleInCrossing", crossingArea, obstacleInCrossing);
		return obstacleInCrossing;
	}

	private CarPositions analyzeCarPositions(DrivePoint inPoint)
	{
		SegmentLink inLink = inPoint.getGoalLink();
		Segment crossing = inLink.getSegmentAfter();

		Polygon west = getCrossingOptionArea(crossing, Direction.getLeft(inLink.getDirection()));
		Polygon north = getCrossingOptionArea(crossing, inLink.getDirection());
		Polygon east = getCrossingOptionArea(crossing, Direction.getRight(inLink.getDirection()));

		boolean isCarNorth = false;
		boolean isCarEast = false;
		boolean isCarWest = false;

		for (Obstacle car : getWorldModel().getRecognizedObjects(RecognizedObjectType.CAR)) {
			Polygon carBounds = new Polygon(car.getArea());

			if (north != null && north.intersects(carBounds)) {
				isCarNorth = true;
			}
			if (east != null && east.intersects(carBounds)) {
				isCarEast = true;
			}
			if (west != null && west.intersects(carBounds)) {
				isCarWest = true;
			}
		}

		drawCheckArea("north", north, isCarNorth);
		drawCheckArea("east", east, isCarEast);
		drawCheckArea("west", west, isCarWest);

		return new CarPositions(isCarNorth, isCarEast, isCarWest);
	}

	private Polygon getCrossingOptionArea(Segment crossing, Direction inOption)
	{
		if (crossing.hasInOption(inOption)) {
			return CROSSING_OPTION_AREA.transform(crossing.getInOption(inOption).getPose());
		}
		return null;
	}

	private void drawCheckArea(String name, Polygon crossingArea, boolean containsObstacle)
	{
		if (crossingArea == null) {
			getThoughtModel().getDrawings().remove(name);
		} else {
			getThoughtModel().getDrawings().draw(
					name, containsObstacle ? new Color(255, 0, 0, 75) : new Color(0, 255, 0, 75), crossingArea);
		}
	}
}
