package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.Polygon;
import hso.autonomy.util.logging.DrawingMap;
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
import java.util.function.BiFunction;

/** Behavior that waits until we have right of way */
public class GiveWay extends AudiCupBehavior
{
	/** the time we wait at a crossing before checking again if we have to give way (in s) */
	private static final float GIVE_WAY_WAIT_TIME = 3.0f;

	private enum Phase { CHECKING_ACTION, WAITING, FINISHED }

	private final IRightOfWayAnalyzer rightOfWayAnalyzer = new RightOfWayAnalyzer();

	private Phase phase;

	/** the time we started waiting before checking decision again */
	private float waitStartTime;

	/** true if we have stopped already at a stop and wait crossing */
	private boolean haveStopped;

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
			RightOfWayAction action = determineRightOfWayAction(path.get(1), path.get(2));

			if (action == RightOfWayAction.DRIVE) {
				phase = Phase.FINISHED;
			} else if (action == RightOfWayAction.STOP_THEN_DRIVE && haveStopped) {
				phase = Phase.FINISHED;
			} else {
				getAgentModel().getMotor().stop();
				waitStartTime = getWorldModel().getGlobalTime();
				haveStopped = true;
				phase = Phase.WAITING;
			}

			break;

		case WAITING:
			if (getWorldModel().getGlobalTime() - waitStartTime >= GIVE_WAY_WAIT_TIME) {
				// we check again next cycle
				phase = Phase.CHECKING_ACTION;
			}
			break;
		}
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
		drawCrossingArea("obstacleInCrossing", crossingArea, obstacleInCrossing);
		return obstacleInCrossing;
	}

	private CarPositions analyzeCarPositions(DrivePoint inPoint)
	{
		SegmentLink inLink = inPoint.getGoalLink();
		Segment crossing = inLink.getSegmentAfter();
		SegmentLink west = crossing.getInOption(Direction.getLeft(inLink.getDirection()));
		SegmentLink north = crossing.getInOption(inLink.getDirection());
		SegmentLink east = crossing.getInOption(Direction.getRight(inLink.getDirection()));

		Polygon crossingOptionArea = new Polygon(new Area2D.Float(-0.7, 0.1, -0.25, 0.25));

		boolean isCarNorth = false;
		boolean isCarEast = false;
		boolean isCarWest = false;

		for (Obstacle car : getWorldModel().getRecognizedObjects(RecognizedObjectType.CAR)) {
			Polygon polygon = new Polygon(car.getArea());

			BiFunction<String, SegmentLink, Boolean> checkOption = (name, option) ->
			{
				DrawingMap drawings = getThoughtModel().getDrawings();
				if (option != null) {
					Polygon optionArea = crossingOptionArea.transform(option.getPose());
					boolean collision = polygon.intersects(optionArea);
					drawCrossingArea(name, optionArea, collision);
					return collision;
				} else {
					drawings.remove(name);
					return false;
				}
			};

			isCarNorth = isCarNorth || checkOption.apply("north", north);
			isCarEast = isCarEast || checkOption.apply("east", east);
			isCarWest = isCarWest || checkOption.apply("west", west);
		}

		return new CarPositions(isCarNorth, isCarEast, isCarWest);
	}

	private void drawCrossingArea(String name, Polygon crossingArea, boolean containsObstacle)
	{
		getThoughtModel().getDrawings().draw(
				name, containsObstacle ? new Color(255, 0, 0, 75) : new Color(0, 255, 0, 75), crossingArea);
	}
}
