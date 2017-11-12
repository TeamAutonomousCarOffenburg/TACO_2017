package taco.util.rightofway;

import taco.util.SignType;

public interface IRightOfWayAnalyzer {
	Situation analyzeSituation(CrossingType crossingType, SignType visibleRoadSign, CarPositions carPositions,
			DriveDestination destination);
}
