package taco.util.rightofway;

import taco.util.SignType;

import static taco.util.rightofway.CrossingType.*;

public class RightOfWayAnalyzer implements IRightOfWayAnalyzer
{
	public Situation analyzeSituation(CrossingType crossingType, SignType visibleRoadSign, CarPositions carPositions,
			DriveDestination destination)
	{
		CarPositions rightOfWay = checkRightOfWay(crossingType, visibleRoadSign, carPositions, destination);
		RightOfWayAction action = checkAction(crossingType, visibleRoadSign, destination, rightOfWay);
		return new Situation(crossingType, visibleRoadSign, carPositions, destination, rightOfWay, action);
	}

	private CarPositions checkRightOfWay(CrossingType crossingType, SignType visibleRoadSign, CarPositions carPositions,
			DriveDestination destination)
	{
		boolean north = false;
		boolean east = false;
		boolean west = false;

		// Treat UNKNOWN / "none" as CROSSING
		if (visibleRoadSign == null || visibleRoadSign == SignType.UNKNOWN) {
			visibleRoadSign = SignType.UNMARKED_INTERSECTION;
		}

		// Set rightOfWay (for all crossing types)
		switch (visibleRoadSign) {
		case UNMARKED_INTERSECTION:
			return checkRightBeforeLeftRightOfWay(crossingType, carPositions, destination);
		case GIVE_WAY:
		case STOP:
			north = carPositions.north;
			east = carPositions.east;
			west = carPositions.west;
			break;
		case HAVE_WAY:
			if (destination == DriveDestination.WEST) {
				north = carPositions.north;
			}
			break;
		}

		return new CarPositions(north, east, west);
	}

	private CarPositions checkRightBeforeLeftRightOfWay(
			CrossingType crossingType, CarPositions carPositions, DriveDestination destination)
	{
		boolean north = false;
		boolean east = false;
		boolean west = false;

		if (carPositions.north && !carPositions.east && !carPositions.west) {
			if (destination == DriveDestination.WEST) {
				north = true;
			}
		} else if (carPositions.north && !carPositions.east) {
			if (destination == DriveDestination.WEST) {
				north = true;

				// Special case (see documentation)
				if (crossingType == NORTH_EAST_SOUTH_WEST) {
					west = true;
				}
			}
		} else if (!carPositions.north && carPositions.east && !carPositions.west) {
			east = true;
		} else if (!carPositions.north && carPositions.east) {
			east = true;
		} else if (carPositions.north && !carPositions.west) {
			north = true;
			east = true;
		} else if (carPositions.north) {
			north = true;
			east = true;
			west = true;
		}

		return new CarPositions(north, east, west);
	}

	private RightOfWayAction checkAction(
			CrossingType crossingType, SignType visibleRoadSign, DriveDestination destination, CarPositions rightOfWay)
	{
		if (crossingType == EAST_SOUTH_WEST) {
			if (destination == DriveDestination.NORTH) {
				return RightOfWayAction.NOT_PERMITTED;
			}
		} else if (crossingType == NORTH_EAST_SOUTH) {
			if (destination == DriveDestination.WEST) {
				return RightOfWayAction.NOT_PERMITTED;
			}
		} else if (crossingType == NORTH_SOUTH_WEST) {
			if (destination == DriveDestination.EAST) {
				return RightOfWayAction.NOT_PERMITTED;
			}
		}

		if (rightOfWay.east || rightOfWay.north || rightOfWay.west) {
			return RightOfWayAction.WAIT;
		} else {
			if (visibleRoadSign == SignType.STOP) {
				return RightOfWayAction.STOP_THEN_DRIVE;
			} else {
				return RightOfWayAction.DRIVE;
			}
		}
	}
}
