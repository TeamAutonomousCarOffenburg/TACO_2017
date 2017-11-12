package taco.util.rightofway;

import taco.util.SignType;

public class Situation
{
	public final CrossingType crossingType;

	public final SignType visibleRoadSign;

	public final CarPositions carPositions;

	public final DriveDestination destination;

	public final CarPositions rightOfWay;

	public final RightOfWayAction action;

	public Situation(CrossingType crossingType, SignType visibleRoadSign, CarPositions carPositions,
			DriveDestination destination, CarPositions rightOfWay, RightOfWayAction action)
	{
		this.crossingType = crossingType;
		this.visibleRoadSign = visibleRoadSign;
		this.carPositions = carPositions;
		this.destination = destination;
		this.rightOfWay = rightOfWay;
		this.action = action;
	}
}
