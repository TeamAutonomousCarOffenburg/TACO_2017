package taco.agent.model.worldmodel.street.maps;

import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.agent.model.worldmodel.street.Direction;
import taco.agent.model.worldmodel.street.ParkingSegment;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;
import taco.agent.model.worldmodel.street.SegmentType;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.util.SignType;

/**
 * Static utils for Maps
 */
public class MapUtils
{
	private static final Pose2D signTranslation = new Pose2D(-0.1, -0.2);

	public static Segment appendStraights(Segment segment, Direction dir, int n)
	{
		return appendStraights(segment, dir, n, IAudiCupMotor.DEFAULT_SPEED);
	}

	public static Segment appendStraights(Segment segment, Direction dir, int n, double speed)
	{
		return appendStraights(segment, dir, n, speed, speed);
	}

	public static Segment appendStraights(
			Segment segment, Direction dir, int n, double speedOurLane, double speedOtherLane)
	{
		for (int i = 0; i < n; i++) {
			segment = segment.getOutOption(dir).appendStraightSegment(speedOurLane, speedOtherLane);
		}
		return segment;
	}

	public static Segment appendCrosswalk(Segment segment, Direction dir)
	{
		segment = segment.getOutOption(dir).appendStraightSegmentWithCrosswalk(
				1, IAudiCupMotor.LOW_SPEED, IAudiCupMotor.LOW_SPEED);
		addSign(segment, dir, SignType.CROSSWALK);
		addSign(segment, Direction.getOppositeDirection(dir), SignType.CROSSWALK);
		return segment;
	}

	public static void addSign(Segment segment, Direction dir, SignType type)
	{
		if (segment.hasInOption(dir)) {
			segment.getInOption(dir).addRoadSign(
					new RoadSign(type, segment.getInOption(dir).getPose().applyTo(signTranslation)));
		}
	}

	public static void addParkingVertical(StreetMap map, int segmentID, Direction dir, int firstID, boolean[] occupied)
	{
		addParking(map, segmentID, dir, SegmentType.PARKING_SPACE_VERTICAL, firstID, occupied, 0.25);
	}

	public static void addParkingVertical(
			StreetMap map, int segmentID, Direction dir, int firstID, boolean ascending, boolean[] occupied)
	{
		addParking(map, segmentID, dir, SegmentType.PARKING_SPACE_VERTICAL, firstID, ascending, occupied, 0.25);
	}

	public static void addParkingHorizontal(
			StreetMap map, int segmentID, Direction dir, int firstID, boolean[] occupied)
	{
		addParking(map, segmentID, dir, SegmentType.PARKING_SPACE_HORIZONTAL, firstID, occupied, 0.25);
	}

	public static ParkingSegment addParking(StreetMap map, int segmentID, Direction dir, SegmentType type, int firstID,
			boolean[] occupied, double translation)
	{
		return addParking(map, segmentID, dir, type, firstID, true, occupied, translation);
	}

	public static ParkingSegment addParking(StreetMap map, int segmentID, Direction dir, SegmentType type, int firstID,
			boolean ascending, boolean[] occupied, double translation)
	{
		Segment segment = map.getSegment(segmentID);
		SegmentLink parking = segment.getInOption(dir);
		ParkingSegment parkingSegment =
				new ParkingSegment(map, type, firstID, ascending, parking, translation, occupied);
		parking.getSegmentAfter().setAttachedOption(parkingSegment.getInOptionByParkingID(firstID));
		addSign(segment, dir, SignType.PARKING_AREA);
		return parkingSegment;
	}
}
