package taco.agent.model.worldmodel.signdetection;

import hso.autonomy.util.geometry.*;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import taco.agent.model.worldmodel.street.*;
import taco.agent.model.worldmodel.street.maps.MapUtils;
import taco.util.SignType;
import taco.util.drive.AngleUtils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class RoadSignUtils
{
	/**
	 * If we detect a roadsign, we have to compare it with those of the roadsign.xml and update it when it is a new one
	 *
	 * @param roadSigns all known roadsigns from the given type
	 * @param detectedRoadSign new detected roadsign to find a match
	 * @return true if found the same, false if not (this means the sign was switched by the jury)
	 */
	public static RoadSign findClosestRoadSign(List<RoadSign> roadSigns, RoadSign detectedRoadSign)
	{
		for (RoadSign sign : roadSigns) {
			if (sign.getPose().getDistanceTo(detectedRoadSign.getPose()) < 0.6) {
				return sign;
			}
		}
		return null;
	}

	public static RoadSign findClosestRoadSign(List<RoadSign> allSigns, Pose2D carPose)
	{
		allSigns.sort(new RoadSignComparator(carPose));
		return allSigns.get(0);
	}

	public static List<RoadSign> getSignsByType(List<RoadSign> allSigns, SignType type)
	{
		List<RoadSign> signsByType = new ArrayList<>();
		for (RoadSign sign : allSigns) {
			if (sign.getSignType() == type) {
				signsByType.add(sign);
			}
		}
		return signsByType;
	}

	/**
	 * @param map the used map with all required segments
	 * @param signs all known signs (stored in environment manager)
	 * @param sign current detected sign
	 * @return true if all criteria were passed and the sign is stored to the map
	 */
	public static boolean loadReceivedRoadSignIntoMap(
			StreetMap map, List<RoadSign> signs, RoadSign sign, float globalTime)
	{
		if (checkForExistingSignsFromSameType(signs, sign, globalTime)) {
			return false;
		}
		// if the direction is SOUTH, it would mean we would see the backside of the sign -> invalid angle from
		// detection
		Direction direction = Direction.getDirection(sign.getPose().getAngle());
		if (direction.equals(Direction.SOUTH)) {
			return false;
		}

		Vector3D signPosition = sign.getPose().getPosition();
		// get the segment that contains the recognized sign
		Segment segmentOfSign = map.getSegmentContaining(signPosition);
		List<Segment> checkedSegments = new ArrayList<>();
		while (true) {
			if (segmentOfSign != null) {
				// don't match crosswalk sign to a non-crosswalk segment
				if (!(segmentOfSign.getType() == SegmentType.STRAIGHT_WITH_CROSSWALK) &&
						sign.getSignType() == SignType.CROSSWALK) {
					return false;
				}
				checkedSegments.add(segmentOfSign);
				// get closest segmentlink to find the correct one to add roadsign
				SegmentLink closestLinkToPose = segmentOfSign.getClosestLinkToPose(sign.getPose());
				if (checkForExistingSignAtPosition(sign, closestLinkToPose) || segmentOfSign.hasOutOption(direction)) {
					MapUtils.addSign(closestLinkToPose.getSegmentAfter(),
							Direction.getDirection(closestLinkToPose.getDirection().getAngle().getAdjacencyAngle()),
							sign.getSignType());
					return true;
				} else {
					// check for alternate segment if no match could be found
					segmentOfSign = map.getSegmentContainingOnlyXorY(signPosition, checkedSegments);
				}
			} else {
				// get alternate segment if getSegmentContaining(Vector3D) doesn't found one
				segmentOfSign = map.getSegmentContainingOnlyXorY(signPosition, checkedSegments);
			}
			if (segmentOfSign == null) {
				return false;
			}
		}
	}

	private static boolean checkForExistingSignAtPosition(RoadSign sign, SegmentLink closestLinkToPose)
	{
		if (closestLinkToPose.getRoadSigns().isEmpty()) {
			return false;
		}
		RoadSign matchedSign = null;
		for (RoadSign signInLink : closestLinkToPose.getRoadSigns()) {
			if (sign.getPose().getDistanceTo(signInLink.getPose()) < 0.7) {
				matchedSign = signInLink;
			}
		}
		// remove "old" sign if this link already had one to account the current one
		if (matchedSign != null) {
			closestLinkToPose.getRoadSigns().remove(matchedSign);
			return true;
		}
		return false;
	}

	private static boolean checkForExistingSignsFromSameType(List<RoadSign> signs, RoadSign sign, float globalTime)
	{
		signs = getSignsByType(signs, sign.getSignType());
		for (RoadSign filteredSign : signs) {
			// if we already know a roadsign near the detected position, ignore the detection
			if (sign.getPose().getDistanceTo(filteredSign.getPose()) < 0.7) {
				filteredSign.update(true, true, globalTime);
				return true;
			}
		}
		return false;
	}

	public static void removeSignFromMap(StreetMap map, RoadSign removedSign)
	{
		Direction direction = Direction.getDirection(removedSign.getPose().getAngle());
		Segment segmentToRemoveSign = map.getSegmentContaining(removedSign.getPose().getPosition());
		if (segmentToRemoveSign.hasOutOption(direction)) {
			segmentToRemoveSign.getOutOption(direction).getRoadSigns().remove(removedSign);
		}
	}

	/**
	 * Checks if the sign is in the visible area. This means the distance should be less than 2.5m and the angle between
	 * [-40°;40°] in front of the car.
	 */
	public static boolean isInVisibleArea(IPose2D globalCameraPose, RoadSign sign)
	{
		Angle angleToSign = globalCameraPose.getAngleTo(sign.getPose());
		Direction carDir = Direction.getDirection(globalCameraPose.getAngle());
		Direction signDir = Direction.getDirection(sign.getPose().getAngle());
		Polygon visibleArea = getvisibleArea(globalCameraPose);

		// this condition means it is inside the visible area..
		if (visibleArea.contains(VectorUtils.to2D(sign.getPose().getPosition()))) {
			// but we have to filter the signs that are inside the area but caused by the direction it can't be visible
			// e.g. signs in crossings
			if ((angleToSign.degrees() < 0 && Direction.getLeft(carDir) == Direction.getOppositeDirection(signDir)) ||
					(angleToSign.degrees() > 0 &&
							Direction.getRight(carDir) == Direction.getOppositeDirection(signDir)) ||
					(carDir == signDir)) {
				return true;
			}
		}
		return false;
	}

	public static Polygon getvisibleArea(IPose2D globalCameraPose)
	{
		Vector2D leftLimit = AngleUtils.getVectorFromAngle(Math.toRadians(-35), 2.5);
		Vector2D rightLimit = AngleUtils.getVectorFromAngle(Math.toRadians(35), 2.5);
		Vector2D globalLeftLimit = globalCameraPose.applyTo(leftLimit);
		Vector2D globalRightLimit = globalCameraPose.applyTo(rightLimit);

		return new Polygon(VectorUtils.to2D(globalCameraPose.getPosition()), globalLeftLimit, globalRightLimit);
	}

	/**
	 * This comparator is used to sort the list of roadsigns by the distance to the given car position
	 */
	private static class RoadSignComparator implements Comparator<RoadSign>
	{
		private Pose2D currentCarPose;

		public RoadSignComparator(Pose2D currentCarPose)
		{
			this.currentCarPose = currentCarPose;
		}

		@Override
		public int compare(RoadSign o1, RoadSign o2)
		{
			double distanceToSign1 = currentCarPose.getDistanceTo(o1.getPose());
			double distanceToSign2 = currentCarPose.getDistanceTo(o2.getPose());
			return Double.compare(distanceToSign1, distanceToSign2);
		}
	}
}
