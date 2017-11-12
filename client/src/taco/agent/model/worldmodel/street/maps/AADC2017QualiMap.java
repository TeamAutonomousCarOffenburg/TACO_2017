package taco.agent.model.worldmodel.street.maps;

import static taco.agent.model.agentmodel.IAudiCupMotor.DEFAULT_SPEED;
import static taco.agent.model.agentmodel.IAudiCupMotor.HIGH_SPEED;
import static taco.agent.model.agentmodel.IAudiCupMotor.LOW_SPEED;
import static taco.agent.model.worldmodel.street.Direction.EAST;
import static taco.agent.model.worldmodel.street.Direction.NORTH;
import static taco.agent.model.worldmodel.street.Direction.SOUTH;
import static taco.agent.model.worldmodel.street.Direction.WEST;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addParkingVertical;
import static taco.agent.model.worldmodel.street.maps.MapUtils.addSign;
import static taco.agent.model.worldmodel.street.maps.MapUtils.appendCrosswalk;
import static taco.agent.model.worldmodel.street.maps.MapUtils.appendStraights;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.street.Direction;
import taco.agent.model.worldmodel.street.NonCrossingSegment;
import taco.agent.model.worldmodel.street.Sector;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.util.SignType;

/**
 * Map used during the AADC 2017 test event
 */
public class AADC2017QualiMap
{
	public static StreetMap create()
	{
		Segment.nextID = 0;
		Segment rootSegment = NonCrossingSegment.createInitialSegment(new Pose2D(2, 11.5), 1, Angle.ZERO);

		// middle segments
		Segment current = rootSegment.getOutOption(SOUTH).appendCurveSmallLeft();
		current = appendStraights(current, EAST, 1);
		Segment c1 = current.getOutOption(EAST).appendTCrossingLS();
		current = appendStraights(c1, EAST, 2);
		Segment c2 = current.getOutOption(EAST).appendTCrossingLS();
		current = appendStraights(c2, EAST, 2);
		current = current.getOutOption(EAST).appendCurveBigLeft();
		current = appendStraights(current, NORTH, 1);
		current = appendStraights(current, NORTH, 10, DEFAULT_SPEED);
		current = appendStraights(current, NORTH, 1, LOW_SPEED);
		Segment c3 = current.getOutOption(NORTH).appendTCrossingLS();
		current = appendStraights(c3, NORTH, 1, LOW_SPEED);
		current = appendStraights(current, NORTH, 3);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = appendStraights(current, WEST, 3);
		current = appendStraights(current, WEST, 1, LOW_SPEED, DEFAULT_SPEED);
		current = appendCrosswalk(current, WEST);
		current = appendStraights(current, WEST, 1, DEFAULT_SPEED, LOW_SPEED);
		current = appendStraights(current, WEST, 2);
		current = current.getOutOption(WEST).appendCurveSmallLeft();
		current = appendStraights(current, SOUTH, 1);
		current = appendStraights(current, SOUTH, 7, HIGH_SPEED);
		current = appendStraights(current, SOUTH, 2, LOW_SPEED, HIGH_SPEED);
		current = appendCrosswalk(current, SOUTH);
		current = appendStraights(current, SOUTH, 2, DEFAULT_SPEED, LOW_SPEED);
		Segment c4 = current.getOutOption(SOUTH).appendTCrossingLS();
		current = appendStraights(c4, SOUTH, 2);
		Segment c5 = current.getOutOption(SOUTH).appendTCrossingLS();
		c5.getOutOption(SOUTH).connectToInLink(rootSegment.getInOption(NORTH));

		// interior crossings
		current = appendStraights(c5, EAST, 2);
		Segment c6 = current.getOutOption(EAST).appendXCrossing();
		current = appendStraights(c6, EAST, 2);
		Segment c7 = current.getOutOption(EAST).appendXCrossing();
		current = appendStraights(c7, NORTH, 1);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = appendStraights(current, WEST, 1);
		Segment c8 = current.getOutOption(WEST).appendTCrossingLS();
		current = appendStraights(c8, WEST, 2);
		current.getOutOption(WEST).connectToInLink(c4.getInOption(EAST));

		// close rings
		current = appendStraights(c8, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c6.getInOption(NORTH));
		current = appendStraights(c6, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c1.getInOption(NORTH));
		current = appendStraights(c7, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c2.getInOption(NORTH));

		// s curves
		current = appendStraights(c7, EAST, 1);
		current = current.getOutOption(EAST).appendCurveSmallLeft();
		current = appendStraights(current, NORTH, 1);
		current = current.getOutOption(NORTH).appendSCurveBottom();
		current = appendStraights(current, NORTH, 1);
		current = current.getOutOption(NORTH).appendSCurveBottom();
		current = current.getOutOption(NORTH).appendSCurveBottom();
		current = appendStraights(current, NORTH, 1);
		current = current.getOutOption(NORTH).appendCurveBigRight();
		current = current.getOutOption(EAST).appendCurveBigRight();
		current = current.getOutOption(SOUTH).appendCurveSmallLeft();
		current = appendStraights(current, EAST, 1, LOW_SPEED);
		current.getOutOption(EAST).connectToInLink(c3.getInOption(WEST));

		// create the map
		StreetMap map = new StreetMap(rootSegment);

		// add traffic signs
		addSign(c1, NORTH, SignType.GIVE_WAY);
		addSign(c1, EAST, SignType.HAVE_WAY);
		addSign(c2, NORTH, SignType.STOP);
		addSign(c2, EAST, SignType.HAVE_WAY);
		addSign(c3, NORTH, SignType.HAVE_WAY);
		addSign(c3, SOUTH, SignType.HAVE_WAY);
		addSign(c3, WEST, SignType.GIVE_WAY);
		addSign(c4, NORTH, SignType.HAVE_WAY);
		addSign(c4, SOUTH, SignType.HAVE_WAY);
		addSign(c4, EAST, SignType.STOP);
		addSign(c5, NORTH, SignType.HAVE_WAY);
		addSign(c5, SOUTH, SignType.HAVE_WAY);
		addSign(c5, EAST, SignType.STOP);

		addSign(c6, SOUTH, SignType.HAVE_WAY);
		addSign(c6, NORTH, SignType.HAVE_WAY);
		addSign(c6, EAST, SignType.STOP);
		addSign(c6, WEST, SignType.STOP);
		addSign(c7, EAST, SignType.HAVE_WAY);
		addSign(c7, WEST, SignType.HAVE_WAY);
		addSign(c7, NORTH, SignType.STOP);
		addSign(c7, SOUTH, SignType.STOP);
		addSign(c8, EAST, SignType.HAVE_WAY);
		addSign(c8, WEST, SignType.HAVE_WAY);
		addSign(c8, SOUTH, SignType.STOP);

		// create parking spaces
		boolean[] occupied1 = {true, false, true, false};
		addParkingVertical(map, 18, SOUTH, 1, occupied1);

		boolean[] occupied2 = {true, false, true, false};
		addParkingVertical(map, 43, SOUTH, 5, occupied2);

		// add stop lines
		map.getSegment(54).getOutOption(Direction.WEST).addStopLine();
		map.getSegment(55).getOutOption(Direction.EAST).addStopLine();
		map.getSegment(65).getOutOption(Direction.WEST).addStopLine();
		map.getSegment(66).getOutOption(Direction.NORTH).addStopLine();
		map.getSegment(57).getOutOption(Direction.WEST).addStopLine();
		map.getSegment(60).getOutOption(Direction.SOUTH).addStopLine();
		map.getSegment(71).getOutOption(Direction.SOUTH).addStopLine();
		map.getSegment(70).getOutOption(Direction.NORTH).addStopLine();

		// create sectors
		map.addSector(new Sector(45, map.getSegment(45).getOutOption(SOUTH).getPose()));
		map.addSector(new Sector(24, map.getSegment(24).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(3, map.getSegment(3).getOutOption(EAST).getPose()));
		map.addSector(new Sector(15, map.getSegment(15).getOutOption(NORTH).getPose()));

		return map;
	}
}
