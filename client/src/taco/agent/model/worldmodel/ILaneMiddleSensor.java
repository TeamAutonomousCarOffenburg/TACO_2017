package taco.agent.model.worldmodel;

import java.util.List;

import hso.autonomy.util.geometry.IPose2D;
import taco.agent.communication.perception.impl.LaneMiddlePerceptor;
import taco.agent.model.agentmodel.ICameraSensor;
import taco.agent.model.worldmodel.impl.Obstacle;
import taco.agent.model.worldmodel.lanedetection.HistoryEntry;
import taco.agent.model.worldmodel.lanedetection.LaneMiddle;
import taco.agent.model.worldmodel.street.RuntimeSegment;
import taco.agent.model.worldmodel.street.StreetMap;

public interface ILaneMiddleSensor {
	/**
	 * Updates the sensor with new perceptor readings
	 * @param perceptor the lane middle perceptor (not null)
	 * @param time the current global time
	 * @param carPose the current (one cycle behind) pose of the car
	 * @param laneMiddles lane middle objects from object detection
	 * @param map the street map to match with
	 */
	void update(LaneMiddlePerceptor perceptor, float time, IPose2D carPose, List<Obstacle> laneMiddles, StreetMap map);

	/**
	 * Checks if we can reposition side wise and angle wise
	 * @param currentSegment the segment we drive in
	 * @param time the current global time (in s)
	 * @param carPose the current pose of the car
	 * @return the new pose of the car, null if not possible
	 */
	IPose2D calculateLateralRepositioning(RuntimeSegment currentSegment, float time, IPose2D carPose);

	/**
	 * Checks if we have information of a stop line to reposition in driving direction
	 * @param currentSegment the segment we drive in
	 * @param time the current global time (in s)
	 * @param carPose the current pose of the car
	 * @return the new pose of the car, null if not possible
	 */
	IPose2D calculateSagittalRepositioning(
			RuntimeSegment currentSegment, float time, IPose2D carPose, List<Obstacle> stopLines, ICameraSensor camera);

	/**
	 * Checks if we can reposition side wise when switching to a new segment
	 * @param currentSegment the segment we drive in
	 * @param carPose the current pose of the car
	 * @param delta the delta by which we require to be inside the segment (in m)
	 * @return the new pose of the car, null if not possible
	 */
	IPose2D calculateLateralRepositioningOnSegmentChange(RuntimeSegment currentSegment, IPose2D carPose, double delta);

	/**
	 * @return the pixel difference of expected and real lane middle x coordinate
	 */
	int getDeltaX();

	int getLeftDeltaX();

	int getLeftMiddleX();

	LaneMiddle getCurrent();

	boolean isReliable(double requiredConfidence);

	void updateConsecutiveFollowRightLanePerforms(int performs);

	int getHistorySize();

	HistoryEntry getHistoryEntry(int index);
}