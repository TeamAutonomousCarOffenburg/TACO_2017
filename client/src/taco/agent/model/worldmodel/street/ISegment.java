package taco.agent.model.worldmodel.street;

public interface ISegment {
	double STREET_WIDTH = 0.9;

	double LANE_WIDTH = STREET_WIDTH * 0.5;

	double LANE_HALF_WIDTH = LANE_WIDTH * 0.5;

	double CROSSING_LENGTH = 1.0;

	double CROSSING_HALF_LENGTH = CROSSING_LENGTH * 0.5;
}
