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
import taco.agent.model.worldmodel.street.NonCrossingSegment;
import taco.agent.model.worldmodel.street.Sector;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.util.SignType;

/**
 * Map used during the AADC 2017 test event
 */
public class AADC2017FinalMap
{
	public static StreetMap create()
	{
		Segment.nextID = 0;
		Segment rootSegment = NonCrossingSegment.createInitialSegment(new Pose2D(2, 0.5), 1, Angle.ZERO);
		rootSegment.getOutOption(NORTH).setSpeed(LOW_SPEED);

		// middle segments
		Segment current = appendCrosswalk(rootSegment, NORTH);
		current = appendStraights(current, NORTH, 3, HIGH_SPEED, LOW_SPEED);
		current = appendStraights(current, NORTH, 2, HIGH_SPEED, DEFAULT_SPEED);
		current = current.getOutOption(NORTH).appendSCurveBottom();
		current = appendStraights(current, NORTH, 1);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = appendStraights(current, WEST, 6, HIGH_SPEED);
		current = appendStraights(current, WEST, 3);
		current = current.getOutOption(WEST).appendCurveBigLeft();
		current = appendStraights(current, SOUTH, 2);
		Segment c1 = current.getOutOption(SOUTH).appendTCrossingLS();
		current = appendStraights(c1, SOUTH, 2);
		Segment c2 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c2, SOUTH, 1);
		current = current.getOutOption(SOUTH).appendCurveSmallLeft();
		current = appendStraights(current, EAST, 1);
		Segment c3 = current.getOutOption(EAST).appendXCrossing();
		current = appendStraights(c3, SOUTH, 1);
		current = current.getOutOption(SOUTH).appendCurveSmallLeft();
		current = appendStraights(current, EAST, 2);
		Segment c4 = current.getOutOption(EAST).appendTCrossingLS();
		current = appendStraights(c4, EAST, 2);
		Segment c5 = current.getOutOption(EAST).appendTCrossingLS();
		current = appendStraights(c5, EAST, 3);
		current = current.getOutOption(EAST).appendCurveSmallLeft();
		current.getOutOption(NORTH).connectToInLink(rootSegment.getInOption(SOUTH));

		// first inner loop
		current = appendStraights(c5, NORTH, 2);
		Segment c6 = current.getOutOption(NORTH).appendXCrossing();
		current = appendStraights(c6, NORTH, 1);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = appendStraights(current, WEST, 1);
		Segment c7 = current.getOutOption(WEST).appendTCrossingLS();
		current = appendStraights(c7, SOUTH, 2);
		Segment c8 = current.getOutOption(SOUTH).appendXCrossing();
		current = appendStraights(c8, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c4.getInOption(NORTH));

		// last crossing
		current = appendStraights(c7, WEST, 3);
		Segment c9 = current.getOutOption(WEST).appendTCrossingLS();
		current = appendStraights(c9, WEST, 2);
		current.getOutOption(WEST).connectToInLink(c2.getInOption(EAST));

		// connections
		current = appendStraights(c9, SOUTH, 2);
		current.getOutOption(SOUTH).connectToInLink(c3.getInOption(NORTH));
		current = appendStraights(c3, EAST, 1, LOW_SPEED, DEFAULT_SPEED);
		current = appendCrosswalk(current, EAST);
		current = appendStraights(current, EAST, 1, DEFAULT_SPEED, LOW_SPEED);
		current.getOutOption(EAST).connectToInLink(c8.getInOption(WEST));
		current = appendStraights(c8, EAST, 2);
		current.getOutOption(EAST).connectToInLink(c6.getInOption(WEST));

		// long curves
		current = appendStraights(c6, EAST, 1);
		current = current.getOutOption(EAST).appendCurveBigLeft();
		current = appendStraights(current, NORTH, 1);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = current.getOutOption(WEST).appendCurveSmallRight();
		current = appendStraights(current, NORTH, 2);
		current = current.getOutOption(NORTH).appendCurveSmallLeft();
		current = appendStraights(current, WEST, 1);
		current = current.getOutOption(WEST).appendSCurveBottom();
		current = current.getOutOption(WEST).appendSCurveBottom();
		current = appendStraights(current, WEST, 2);
		current.getOutOption(WEST).connectToInLink(c1.getInOption(EAST));

		current = appendStraights(c2, WEST, 6);

		// create the map
		StreetMap map = new StreetMap(rootSegment);

		// add traffic signs
		addSign(c1, NORTH, SignType.HAVE_WAY);
		addSign(c1, SOUTH, SignType.HAVE_WAY);
		addSign(c1, EAST, SignType.STOP);
		addSign(c2, WEST, SignType.STOP);
		addSign(c2, NORTH, SignType.HAVE_WAY);
		addSign(c2, SOUTH, SignType.HAVE_WAY);
		addSign(c2, EAST, SignType.STOP);
		addSign(c3, WEST, SignType.HAVE_WAY);
		addSign(c3, NORTH, SignType.STOP);
		addSign(c3, SOUTH, SignType.STOP);
		addSign(c3, EAST, SignType.HAVE_WAY);
		addSign(c4, WEST, SignType.HAVE_WAY);
		addSign(c4, NORTH, SignType.STOP);
		addSign(c4, EAST, SignType.HAVE_WAY);
		addSign(c5, WEST, SignType.HAVE_WAY);
		addSign(c5, NORTH, SignType.STOP);
		addSign(c5, EAST, SignType.HAVE_WAY);

		addSign(c6, NORTH, SignType.HAVE_WAY);
		addSign(c6, EAST, SignType.STOP);
		addSign(c6, WEST, SignType.STOP);
		addSign(c6, SOUTH, SignType.HAVE_WAY);
		addSign(c7, WEST, SignType.HAVE_WAY);
		addSign(c7, SOUTH, SignType.STOP);
		addSign(c7, EAST, SignType.HAVE_WAY);
		addSign(c8, WEST, SignType.HAVE_WAY);
		addSign(c8, NORTH, SignType.STOP);
		addSign(c8, SOUTH, SignType.STOP);
		addSign(c8, EAST, SignType.HAVE_WAY);

		addSign(c9, WEST, SignType.HAVE_WAY);
		addSign(c9, SOUTH, SignType.STOP);
		addSign(c9, EAST, SignType.HAVE_WAY);

		// create parking spaces
		boolean[] occupied1 = {true, false, true, false};
		addParkingVertical(map, 82, EAST, 1, false, occupied1);

		boolean[] occupied2 = {true, false, true, false};
		addParkingVertical(map, 17, WEST, 5, false, occupied2);

		// add stop lines
		map.getSegment(79).getOutOption(WEST).addStopLine();
		map.getSegment(80).getOutOption(EAST).addStopLine();
		map.getSegment(59).getOutOption(WEST).addStopLine();
		map.getSegment(60).getOutOption(NORTH).addStopLine();
		map.getSegment(61).getOutOption(SOUTH).addStopLine();
		map.getSegment(30).getOutOption(NORTH).addStopLine();
		map.getSegment(53).getOutOption(SOUTH).addStopLine();
		map.getSegment(52).getOutOption(NORTH).addStopLine();
		map.getSegment(67).getOutOption(WEST).addStopLine();
		map.getSegment(42).getOutOption(SOUTH).addStopLine();
		map.getSegment(66).getOutOption(EAST).addStopLine();
		map.getSegment(50).getOutOption(SOUTH).addStopLine();
		map.getSegment(49).getOutOption(NORTH).addStopLine();

		// create sectors
		map.addSector(new Sector(86, map.getSegment(86).getOutOption(SOUTH).getPose().applyTo(new Pose2D(-0.1, 0.5))));
		map.addSector(new Sector(26, map.getSegment(26).getOutOption(SOUTH).getPose()));
		map.addSector(new Sector(35, map.getSegment(35).getOutOption(EAST).getPose()));
		map.addSector(new Sector(67, map.getSegment(67).getOutOption(EAST).getPose()));
		map.addSector(new Sector(21, map.getSegment(21).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(42, map.getSegment(42).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(35, map.getSegment(35).getOutOption(EAST).getPose()));
		map.addSector(new Sector(21, map.getSegment(21).getOutOption(NORTH).getPose()));
		map.addSector(new Sector(36, map.getSegment(36).getOutOption(WEST).getPose()));

		return map;
	}
}
