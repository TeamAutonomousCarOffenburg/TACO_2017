package taco.agent.model.worldmodel.street.maps;

import static taco.agent.model.worldmodel.street.Direction.EAST;
import static taco.agent.model.worldmodel.street.Direction.NORTH;
import static taco.agent.model.worldmodel.street.Direction.SOUTH;
import static taco.agent.model.worldmodel.street.Direction.WEST;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addParkingHorizontal;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addParkingVertical;
import static taco.agent.model.worldmodel.street.maps.MapUtils.appendStraights;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.street.NonCrossingSegment;
import taco.agent.model.worldmodel.street.Sector;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.StreetMap;

/**
 * Map used during AADC 2015
 */
public class AADC2015Map
{
	public static StreetMap create()
	{
		Segment.nextID = 0;
		Segment rootSegment = NonCrossingSegment.createInitialSegment(new Pose2D(0, 0.5), 1, Angle.ZERO);

		// middle segments
		Segment current = appendStraights(rootSegment, SOUTH, 3);
		Segment c1 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c1, SOUTH, 2);
		Segment c2 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c2, SOUTH, 2);
		Segment c3 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c3, SOUTH, 2);
		Segment c4 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c4, SOUTH, 2);
		Segment c5 = current.getOutOption(SOUTH).appendTCrossingSR();
		current = appendStraights(c5, SOUTH, 2);
		Segment c6 = current.getOutOption(SOUTH).appendTCrossingLR();

		// east loop
		current = appendStraights(c3, WEST, 3);
		Segment c7 = current.getOutOption(WEST).appendXCrossing();
		current = appendStraights(c7, WEST, 2);
		Segment c8 = current.getOutOption(WEST).appendTCrossingLR();
		current = appendStraights(c8, SOUTH, 2);
		Segment c9 = current.getOutOption(SOUTH).appendTCrossingLS();
		current = appendStraights(c9, EAST, 2);
		Segment c10 = current.getOutOption(EAST).appendXCrossing();
		current = appendStraights(c10, EAST, 3);
		current.getOutOption(EAST).connectToInLink(c4.getInOption(WEST));
		current = appendStraights(c10, NORTH, 2);
		current.getOutOption(NORTH).connectToInLink(c7.getInOption(SOUTH));

		// curves NW
		current = appendStraights(c1, WEST, 2);
		current = current.getOutOption(WEST).appendSCurveBottom();
		current = current.getOutOption(WEST).appendCurveSmallLeft();
		current = appendStraights(current, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c8.getInOption(NORTH));

		current = appendStraights(c2, WEST, 2);
		current = current.getOutOption(WEST).appendCurveSmallLeft();
		current = appendStraights(current, SOUTH, 1);
		current.getOutOption(SOUTH).connectToInLink(c7.getInOption(NORTH));

		// curves NE
		current = appendStraights(c6, WEST, 3);
		current = current.getOutOption(WEST).appendCurveSmallRight();
		current = current.getOutOption(NORTH).appendSCurveBottom();
		current = appendStraights(current, NORTH, 1);
		current.getOutOption(NORTH).connectToInLink(c9.getInOption(SOUTH));

		current = appendStraights(c5, WEST, 2);
		current = current.getOutOption(WEST).appendCurveSmallRight();
		current = appendStraights(current, NORTH, 1);
		current.getOutOption(NORTH).connectToInLink(c10.getInOption(SOUTH));

		// east loop
		current = appendStraights(c3, EAST, 6);
		Segment c12 = current.getOutOption(EAST).appendTCrossingLR();
		current = appendStraights(c12, SOUTH, 2);
		Segment c11 = current.getOutOption(SOUTH).appendTCrossingSR();
		current = appendStraights(c11, WEST, 6);

		// curves SE
		current = appendStraights(c6, EAST, 1);
		current = current.getOutOption(EAST).appendSCurveBottom();
		current = current.getOutOption(EAST).appendCurveBigLeft();
		current = appendStraights(current, NORTH, 1);
		current.getOutOption(NORTH).connectToInLink(c11.getInOption(SOUTH));

		// curves SE
		current = appendStraights(c6, EAST, 1);
		current = current.getOutOption(EAST).appendSCurveBottom();
		current = current.getOutOption(EAST).appendCurveBigLeft();
		current = appendStraights(current, NORTH, 1);
		current.getOutOption(NORTH).connectToInLink(c11.getInOption(SOUTH));

		// curves NE
		current = appendStraights(c1, EAST, 2);
		Segment c13 = current.getOutOption(EAST).appendTCrossingSR();
		current = appendStraights(c13, SOUTH, 1);
		current = current.getOutOption(SOUTH).appendCurveSmallRight();
		current = appendStraights(current, WEST, 1);
		current.getOutOption(WEST).connectToInLink(c2.getInOption(EAST));

		current = appendStraights(c13, EAST, 1);
		current = current.getOutOption(EAST).appendCurveBigRight();
		current = appendStraights(current, SOUTH, 3);
		current.getOutOption(SOUTH).connectToInLink(c12.getInOption(NORTH));

		// create the map
		StreetMap map = new StreetMap(rootSegment);

		// add traffic signs

		// create parking spaces
		boolean[] occupied1 = {false, false, false, false};
		addParkingVertical(map, 1, SOUTH, 1, occupied1);

		boolean[] occupied2 = {true, false, true, false};
		addParkingVertical(map, 60, EAST, 5, occupied2);

		boolean[] occupied3 = {false, false, false, true};
		addParkingHorizontal(map, 69, EAST, 9, occupied3);

		// add stop lines
		map.getSegment(73).getOutOption(WEST).addStopLine();
		map.getSegment(68).getOutOption(EAST).addStopLine();
		map.getSegment(58).getOutOption(WEST).addStopLine();
		map.getSegment(63).getOutOption(EAST).addStopLine();
		map.getSegment(87).getOutOption(WEST).addStopLine();
		map.getSegment(85).getOutOption(NORTH).addStopLine();
		map.getSegment(18).getOutOption(SOUTH).addStopLine();
		map.getSegment(14).getOutOption(NORTH).addStopLine();
		map.getSegment(12).getOutOption(SOUTH).addStopLine();
		map.getSegment(11).getOutOption(NORTH).addStopLine();
		map.getSegment(9).getOutOption(SOUTH).addStopLine();
		map.getSegment(5).getOutOption(NORTH).addStopLine();
		map.getSegment(3).getOutOption(SOUTH).addStopLine();
		map.getSegment(54).getOutOption(EAST).addStopLine();
		map.getSegment(35).getOutOption(EAST).addStopLine();
		map.getSegment(20).getOutOption(EAST).addStopLine();
		map.getSegment(44).getOutOption(EAST).addStopLine();
		map.getSegment(36).getOutOption(SOUTH).addStopLine();
		map.getSegment(57).getOutOption(NORTH).addStopLine();
		map.getSegment(47).getOutOption(SOUTH).addStopLine();
		map.getSegment(37).getOutOption(NORTH).addStopLine();

		// create sectors
		map.addSector(new Sector(93, map.getSegment(93).getOutOption(WEST).getPose().applyTo(new Pose2D(-0.1, 0))));
		map.addSector(new Sector(7, map.getSegment(7).getInOption(SOUTH).getPose()));

		return map;
	}
}
