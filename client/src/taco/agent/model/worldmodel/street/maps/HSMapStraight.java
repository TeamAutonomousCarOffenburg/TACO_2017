package taco.agent.model.worldmodel.street.maps;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.street.*;

import static taco.agent.model.worldmodel.street.Direction.*;
import static taco.agent.model.worldmodel.street.maps.MapUtils.appendStraights;

/**
 * Map we have at the Hochschule Offenburg
 */
public class HSMapStraight
{
	public static StreetMap create()
	{
		Segment.nextID = 0;
		Segment rootSegment = NonCrossingSegment.createInitialSegment(new Pose2D(0, 0), 1, Angle.ZERO);

		Segment current = appendStraights(rootSegment, NORTH, 9);

		StreetMap map = new StreetMap(rootSegment);

		map.addSector(new Sector(0, map.getSegment(0).getOutOption(NORTH).getPose()));

		return map;
	}
}
