package taco.agent.model.worldmodel.lanedetection;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import hso.autonomy.util.geometry.VectorUtils;
import hso.autonomy.util.misc.FuzzyCompare;
import taco.agent.communication.perception.RecognizedObject;
import taco.agent.communication.perception.impl.LaneMiddlePerceptor;
import taco.agent.model.agentmodel.ICameraSensor;
import taco.agent.model.worldmodel.ILaneMiddleSensor;
import taco.agent.model.worldmodel.impl.Obstacle;
import taco.agent.model.worldmodel.street.ISegment;
import taco.agent.model.worldmodel.street.RuntimeSegment;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;
import taco.agent.model.worldmodel.street.StreetMap;

public class LaneMiddleSensor implements ILaneMiddleSensor
{
	public static final double DEFAULT_RELIABILITY_THRESHOLD = 0.4;

	private static final float ALLOW_LATERAL_REPOSITIONING_TIME = 3;

	private static final float ALLOW_SAGITTAL_REPOSITIONING_TIME = 2;

	private static final float KEEP_LANE_DETECTION_IN_MEMORY_TIME = 0.3f;

	/** history of valid, consecutive lane middle detections */
	private List<HistoryEntry> sensorHistory;

	/** the latest detection, may be invalid */
	private LaneMiddle current;

	/** time of last lateral repositioning */
	private float lastLateralRepositioningTime;

	/** time of last sagittal repositioning */
	private float lastSagittalRepositioningTime;

	/** time when we saw a lane middle detection from object detection */
	private float lastLaneMiddleSeen;

	/** the last lane middle object detection */
	private RecognizedObject lastLaneMiddle;

	/** the number of times follow lane was performed consecutively */
	private int consecutiveFollowRightLanePerforms;

	public LaneMiddleSensor()
	{
		sensorHistory = new LinkedList<>();
		current = new LaneMiddle();
		lastLateralRepositioningTime = 0;
		lastSagittalRepositioningTime = 0;
		lastLaneMiddleSeen = 0;
		consecutiveFollowRightLanePerforms = 0;
	}

	@Override
	public void update(
			LaneMiddlePerceptor perceptor, float time, IPose2D carPose, List<Obstacle> laneMiddles, StreetMap map)
	{
		current = new LaneMiddle();
		current.update(perceptor, carPose, map);

		// check plausibility with respect to lane middle object detection
		if (!current.isValid()) {
			// TODO: use this line to switch on plausibility check
			// if (!current.isValid() || !isPlausible(time, laneMiddles)) {
			current.setValid(false);
			return;
		}

		// check plausibility with map
		adjustToMap(carPose, map);

		// check if this is a new reading after some time
		//		if (!sensorHistory.isEmpty() && time - sensorHistory.get(0).time > 1.0) {
		//			sensorHistory.clear();
		//		}

		if (!current.isSwapped()) {
			// we do not store swapped positions in history
			if (sensorHistory.isEmpty() || carPose.getDistanceTo(sensorHistory.get(0).carPose) > 0.05) {
				// we only store history every 5 cm
				HistoryEntry newEntry = new HistoryEntry(current, time, carPose);
				sensorHistory.add(0, newEntry);
				if (sensorHistory.size() > 30) {
					// keep history limited
					sensorHistory.remove(sensorHistory.size() - 1);
				}
			}
		}
	}

	private void adjustToMap(IPose2D carPose, StreetMap map)
	{
		// checks if detection might have mixed up right, middle and left lines
		double avgDistanceNew = current.calculateAverageDistance();
		double lastAverageDistance = 0;
		if (!sensorHistory.isEmpty()) {
			lastAverageDistance = sensorHistory.get(0).middle.calculateAverageDistance();
		}

		if (Math.abs(avgDistanceNew - lastAverageDistance) > 0.2) {
			// we have a jump
			if (avgDistanceNew > lastAverageDistance) {
				// try to repair since we seem to get further away from map
				double[] distanceToMapMiddle = current.getDistancesToMapMiddle();

				double laneWidth = 0.45;
				if (distanceToMapMiddle[0] >= 0 && distanceToMapMiddle[1] >= 0 &&
						distanceToMapMiddle[0] < distanceToMapMiddle[1] &&
						FuzzyCompare.eq(current.getGlobalScanPointDistance(0, 1), laneWidth, 0.25)) {
					// mixed up right with middle line
					current.swapRightMiddle();

				} else if (distanceToMapMiddle[2] >= 0 && distanceToMapMiddle[1] >= 0 &&
						   distanceToMapMiddle[2] < distanceToMapMiddle[1] &&
						   FuzzyCompare.eq(current.getGlobalScanPointDistance(2, 1), laneWidth, 0.25)) {
					// mixed up middle with left line
					current.swapMiddleLeft();

				} else {
					// can not be repaired, do not believe
					current.setValid(false);
				}
			}
		}

		if (current.isSwapped()) {
			// positions have changed so recalculate avg distance
			avgDistanceNew = current.calculateAverageDistance();
		}
	}

	private boolean isPlausible(float time, List<RecognizedObject> laneMiddles)
	{
		RecognizedObject lane;
		if (laneMiddles.isEmpty()) {
			if (lastLaneMiddle != null && time - lastLaneMiddleSeen < KEEP_LANE_DETECTION_IN_MEMORY_TIME) {
				// saw a lane recently
				lane = lastLaneMiddle;
			} else {
				// too long since last detection
				return true;
			}
		} else {
			// for now we take the first one, but could look for the lowest highest y value
			lane = laneMiddles.stream().reduce((a, b) -> a.getArea().getMaxY() > b.getArea().getMaxY() ? a : b).get();
			lastLaneMiddle = lane;
			lastLaneMiddleSeen = time;
		}

		int seenX = lane.getArea().getMaxX() - lane.getArea().getMinX();

		int middleLineX = current.getMiddleLineX();
		int halfLaneWidthInPixel = LaneMiddle.cameraToPixel(ISegment.LANE_HALF_WIDTH);
		if (middleLineX < 0) {
			// we did not see the middle line, so guess it from middle of lane
			middleLineX = current.getMiddleX() - halfLaneWidthInPixel;
			if (middleLineX < 0) {
				// the middle is outside the image? we better not believe
				return false;
			}
		}
		if (Math.abs(middleLineX - seenX) > halfLaneWidthInPixel) {
			// line detection too far from object detection
			return false;
		}
		return true;
	}

	@Override
	public IPose2D calculateLateralRepositioning(RuntimeSegment currentSegment, float time, IPose2D carPose)
	{
		if (time - lastLateralRepositioningTime < ALLOW_LATERAL_REPOSITIONING_TIME) {
			// not long enough since last repositioning
			return null;
		}

		if (sensorHistory.size() < 4) {
			return null;
		}

		for (int i = 0; i < 4; i++) {
			HistoryEntry historyEntry = sensorHistory.get(i);
			if (Math.abs(historyEntry.middle.getDeltaX()) > 7) {
				// we drive non straight
				return null;
			}
		}

		if (!currentSegment.isStraight()) {
			// we only reposition if on a straight segment
			return null;
		}

		// we drive 20cm straight now, calculate pose based on current segment
		IPose2D inPose = currentSegment.getInLink().getPose();
		double distance = carPose.getDistanceTo(inPose);
		Angle angle = inPose.getAngle();
		Vector2D offset = angle.applyTo(new Vector2D(distance, 0));
		// TODO: why does this IPose2D return a 3D vector???
		Vector2D newPosition = VectorUtils.to2D(inPose.getPosition()).add(offset);
		Angle carAngle = carPose.getAngle();
		angle = getAngleAdjustment(angle, carAngle, 20, 1.0);
		if (angle == null) {
			return null;
		}

		IPose2D newPose = new Pose2D(newPosition, angle);
		if (newPose.getDistanceTo(carPose) > 0.5) {
			// do not allow large repositioning
			return null;
		}

		lastLateralRepositioningTime = time;
		// System.out.println("Reposition. Old: " + carPose + " new: " + newPose);
		return newPose;
	}

	@Override
	public IPose2D calculateLateralRepositioningOnSegmentChange(
			RuntimeSegment currentSegment, IPose2D carPose, double delta)
	{
		if (consecutiveFollowRightLanePerforms < 30) {
			return null;
		}

		// we are confident that after so many follow lane performs we are in the middle of the track
		IPose2D mapPose = currentSegment.getInLink().getPose();
		if (mapPose.getDistanceTo(carPose) > 0.5) {
			// do not allow large repositioning
			return null;
		}

		Angle newAngle = carPose.getAngle();
		if (currentSegment.isStraight() && currentSegment.getInLink().getSegmentBefore().isStraight()) {
			// when switching between two straights, we also reset the angle
			newAngle = getAngleAdjustment(mapPose.getAngle(), newAngle, 10, 1.0);
			if (newAngle == null) {
				return null;
			}
		}

		// we reposition slightly into the new segment, since the test does the same
		// mapPose = mapPose.applyTo(new Pose2D(delta, 0));

		// we do only sidewise adjustment
		IPose2D localCarPose = mapPose.applyInverseTo(carPose);
		IPose2D adjustedLocalCarPose = new Pose2D(localCarPose.getX(), 0, localCarPose.getAngle());
		IPose2D newPose = mapPose.applyTo(adjustedLocalCarPose);

		return new Pose2D(newPose.getX(), newPose.getY(), newAngle);
	}

	@Override
	public IPose2D calculateSagittalRepositioning(
			RuntimeSegment currentSegment, float time, IPose2D carPose, List<Obstacle> stopLines, ICameraSensor camera)
	{
		if (stopLines.isEmpty()) {
			return null;
		}

		if (time - lastSagittalRepositioningTime < ALLOW_SAGITTAL_REPOSITIONING_TIME) {
			// not long enough since last repositioning
			return null;
		}

		int aheadLineY = current.getAheadLineY();
		if (!isReliable(DEFAULT_RELIABILITY_THRESHOLD) || aheadLineY < 0) {
			// no reliable stop line
			return null;
		}

		Obstacle stopLine = stopLines.get(0);
		if (aheadLineY < stopLine.getArea().getMinY() || aheadLineY > stopLine.getArea().getMaxY()) {
			// the two stop lines do not match
			return null;
		}

		// find crossing we are heading to
		SegmentLink nextSegmentLink = currentSegment.getIntendedOption();
		Segment nextSegment = nextSegmentLink.getSegmentAfter();
		if (nextSegment != null && !nextSegment.isCrossing()) {
			// look for second next
			if (!nextSegment.isStraight()) {
				return null;
			}
			nextSegmentLink = nextSegment.getOutOption(currentSegment.getIntendedOption().getDirection());

			if (nextSegmentLink == null) {
				return null;
			}

			if (!nextSegmentLink.getSegmentAfter().isCrossing()) {
				return null;
			}
		}

		// calculate distance from pixel y coordinate
		Vector3D position = camera.pixelToCar(new Vector2D(current.getMiddleX(), aheadLineY));
		if (position == null) {
			return null;
		}
		double distance = position.getX();
		IPose2D newPose = nextSegmentLink.getPose().applyTo(new Pose2D(-distance, 0));
		if (newPose.getDistanceTo(carPose) > 0.8 || Math.abs(newPose.getDeltaAngle(carPose).degrees()) > 5) {
			// do not allow large repositioning
			return null;
		}

		lastSagittalRepositioningTime = time;
		return newPose;
	}

	private Angle getAngleAdjustment(Angle angle, Angle carAngle, double maxDelta, double maxAdjustment)
	{
		double deltaAngle = Math.abs(carAngle.subtract(angle).degrees());
		if (deltaAngle > maxDelta) {
			return null;
		}
		double deltaAdjust = maxAdjustment;
		if (angle.isRightOf(carAngle)) {
			deltaAdjust = -deltaAdjust;
		}
		if (deltaAngle > maxAdjustment) {
			return carAngle.add(Angle.deg(deltaAdjust));
		}
		return angle;
	}

	@Override
	public int getDeltaX()
	{
		return current.getDeltaX();
	}

	@Override
	public int getLeftDeltaX()
	{
		int deltaX = current.getLeftDeltaX();

		if (deltaX == -1) {
			deltaX = getValidLaneMiddle();

			if (deltaX == -1) {
				return 0;
			}
		}

		return deltaX;
	}

	private int getValidLaneMiddle()
	{
		for (HistoryEntry entry : sensorHistory) {
			int deltaX = entry.middle.getLeftDeltaX();
			if (deltaX >= 0) {
				return deltaX;
			}
		}

		return -1;
	}

	@Override
	public int getLeftMiddleX()
	{
		return current.getLeftMiddleX();
	}

	@Override
	public LaneMiddle getCurrent()
	{
		return current;
	}

	@Override
	public boolean isReliable(double requiredConfidence)
	{
		return current.isValid() && current.getConfidence() >= requiredConfidence;
	}

	@Override
	public void updateConsecutiveFollowRightLanePerforms(int performs)
	{
		consecutiveFollowRightLanePerforms = performs;
	}

	@Override
	public int getHistorySize()
	{
		return sensorHistory.size();
	}

	@Override
	public HistoryEntry getHistoryEntry(int index)
	{
		return sensorHistory.get(index);
	}
}
