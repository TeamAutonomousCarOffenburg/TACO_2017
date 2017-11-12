package taco.agent.model.worldmodel.street.maps;

import static taco.agent.model.worldmodel.street.Direction.EAST;
import static taco.agent.model.worldmodel.street.Direction.NORTH;
import static taco.agent.model.worldmodel.street.Direction.SOUTH;
import static taco.agent.model.worldmodel.street.Direction.WEST;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addParking;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addSign;
import static taco.agent.model.worldmodel.street.maps.MapUtils.appendCrosswalk;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.street.NonCrossingSegment;
import taco.agent.model.worldmodel.street.ParkingSegment;
import taco.agent.model.worldmodel.street.Sector;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentType;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.util.SignType;

/**
 * Map we have at the Hochschule Offenburg
 */
public class HSMap
{
	public static StreetMap create()
	{
		Segment.nextID = 0;
		Segment rootSegment = NonCrossingSegment.createInitialSegment(new Pose2D(0, 0.5), 1, Angle.ZERO);

		Segment straightN = rootSegment.getOutOption(NORTH).appendStraightSegment();
		Segment northCrossing = straightN.getOutOption(NORTH).appendTCrossingLR();
		Segment curve = northCrossing.getOutOption(WEST).appendCurveSmallLeft();
		Segment straightWest = curve.getOutOption(SOUTH).appendStraightSegment();
		curve = northCrossing.getOutOption(EAST).appendCurveSmallRight();
		Segment straightEast = curve.getOutOption(SOUTH).appendStraightSegment();

		Segment westCrossing = straightWest.getOutOption(SOUTH).appendTCrossingLS();
		straightWest = westCrossing.getOutOption(SOUTH).appendStraightSegment();
		Segment eastCrossing = straightEast.getOutOption(SOUTH).appendTCrossingSR();
		straightEast = eastCrossing.getOutOption(SOUTH).appendStraightSegment();
		Segment curveSE = straightEast.getOutOption(SOUTH).appendCurveSmallRight();

		Segment southCrossing = curveSE.getOutOption(WEST).appendTCrossingSR();
		Segment curveSW = southCrossing.getOutOption(WEST).appendCurveSmallRight();

		Segment xCrossing = rootSegment.getOutOption(SOUTH).appendXCrossing();
		// create and connect X crossing with T crossings through straight lanes
		Segment straight = xCrossing.getOutOption(WEST).appendStraightSegment();
		westCrossing.getOutOption(EAST).connectToInLink(straight.getInOption(WEST));

		straight = xCrossing.getOutOption(EAST).appendStraightSegment();
		eastCrossing.getOutOption(WEST).connectToInLink(straight.getInOption(EAST));

		straight = xCrossing.getOutOption(SOUTH).appendStraightSegment();
		straight = appendCrosswalk(straight, SOUTH);
		southCrossing.getOutOption(NORTH).connectToInLink(straight.getInOption(SOUTH));

		// close the ring
		straightWest.getOutOption(SOUTH).connectToInLink(curveSW.getInOption(NORTH));

		// add traffic signs
		addSign(northCrossing, SOUTH, SignType.STOP);
		addSign(xCrossing, SOUTH, SignType.GIVE_WAY);
		addSign(xCrossing, NORTH, SignType.GIVE_WAY);
		addSign(xCrossing, EAST, SignType.HAVE_WAY);
		addSign(xCrossing, WEST, SignType.HAVE_WAY);

		StreetMap map = new StreetMap(rootSegment);

		// add parking spots
		boolean[] occupied1 = {true, false, true, false};
		ParkingSegment parkingSegment =
				addParking(map, 4, NORTH, SegmentType.PARKING_SPACE_VERTICAL, 1, occupied1, 0.75);

		// add stop lines
		map.getSegment(0).getOutOption(SOUTH).addStopLine();
		map.getSegment(1).getOutOption(NORTH).addStopLine();
		map.getSegment(15).getOutOption(EAST).addStopLine();
		map.getSegment(15).getOutOption(WEST).addStopLine();
		map.getSegment(16).getOutOption(EAST).addStopLine();
		map.getSegment(16).getOutOption(WEST).addStopLine();
		map.getSegment(18).getOutOption(SOUTH).addStopLine();

		// define sectors
		map.addSector(new Sector(0, map.getSegment(0).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(3, map.getSegment(3).getOutOption(SOUTH).getPose()));
		map.addSector(new Sector(18, map.getSegment(18).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(4, map.getSegment(4).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(
				parkingSegment.getID(), parkingSegment.getOutOption(EAST).getPose().applyTo(new Pose2D(-0.1, 0))));
		map.addSector(new Sector(8, map.getSegment(8).getOutOption(SOUTH).getPose()));

		return map;
	}
}
