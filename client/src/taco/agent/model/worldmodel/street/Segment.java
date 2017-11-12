package taco.agent.model.worldmodel.street;

import static taco.agent.model.worldmodel.street.ISegment.CROSSING_HALF_LENGTH;
import static taco.agent.model.worldmodel.street.ISegment.LANE_HALF_WIDTH;

import java.util.Collections;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.util.SignType;
import taco.util.rightofway.CrossingType;

/**
 * Class representing a patch of street.
 */
public class Segment
{
	public static int nextID = 0;

	/** unique identifier */
	protected int id;

	/** links into this segment */
	protected SegmentLink[] inOptions;

	/** links out of this segment */
	protected SegmentLink[] outOptions;

	/** Audi Cup type of the segment */
	protected SegmentType type;

	/** specially attached segment like a parking segment */
	protected SegmentLink attachedOption;

	Segment(SegmentType type)
	{
		id = nextID++;
		this.type = type;
		inOptions = new SegmentLink[4];
		outOptions = new SegmentLink[4];
		attachedOption = null;
	}

	public int getID()
	{
		return id;
	}

	public SegmentType getType()
	{
		return type;
	}

	/**
	 * @param direction the (incoming) direction for which to get the road signs
	 * @return all road signs attached to the specified in link
	 */
	public List<RoadSign> getRoadSigns(Direction direction)
	{
		SegmentLink inOption = getInOption(direction);
		if (inOption == null) {
			// should we throw an exception?
			System.err.println("Asking for traffic signs of invalid direction: " + direction + " segment: " + this);
			return Collections.emptyList();
		}
		return inOption.getRoadSigns();
	}

	void addRoadSigns(List<RoadSign> roadSigns, Direction direction)
	{
		SegmentLink inOption = getInOption(direction);
		if (inOption == null) {
			System.err.println("Adding traffic signs to invalid direction: " + direction + " segment: " + this);
		} else {
			inOption.addRoadSigns(roadSigns);
		}
	}

	/**
	 * Checks if this segment has the passed sign type at one of its out options
	 * @param type the type to check
	 * @return true if we have such a sign at an out option
	 */
	public boolean hasRoadSign(SignType type)
	{
		for (Direction current : Direction.values()) {
			if (hasOutOption(current)) {
				SegmentLink outOption = getOutOption(current);
				RoadSign result = outOption.getRoadSigns()
										  .stream()
										  .filter(sign -> sign.getSignType() == type)
										  .findFirst()
										  .orElse(null);
				if (result != null) {
					return true;
				}
			}
		}
		return false;
	}

	public boolean hasOutOption(Direction dir)
	{
		return outOptions[dir.ordinal()] != null;
	}

	public boolean hasInOption(Direction dir)
	{
		return inOptions[dir.ordinal()] != null;
	}

	public boolean hasAttachedOption()
	{
		return attachedOption != null;
	}

	public void setAttachedOption(SegmentLink attached)
	{
		attachedOption = attached;
	}

	public SegmentLink getAttachedOption()
	{
		return attachedOption;
	}

	public SegmentLink getOutOption(Direction dir)
	{
		return outOptions[dir.ordinal()];
	}

	public SegmentLink getInOption(Direction dir)
	{
		return inOptions[dir.ordinal()];
	}

	public SegmentLink getAlternativeInOption(Direction direction)
	{
		// we seek for an in link left or right of passed direction
		SegmentLink result = getInOption(Direction.getLeft(direction));
		if (result == null) {
			// bad luck so right must exist
			result = getInOption(Direction.getRight(direction));
		}
		return result;
	}

	public void setOutOption(Direction dir, SegmentLink link)
	{
		outOptions[dir.ordinal()] = link;
	}

	public void setInOption(Direction dir, SegmentLink link)
	{
		inOptions[dir.ordinal()] = link;
	}

	public boolean isCrossing()
	{
		return true;
	}

	public boolean isCurve()
	{
		return false;
	}

	public boolean isStraight()
	{
		return false;
	}

	public boolean isSCurve()
	{
		return false;
	}

	public boolean isParkingSpace()
	{
		return type.isParkingSpace();
	}

	/**
	 * @return the position of this segment at the south-east corner
	 */
	public Vector2D getPosition()
	{
		if (hasInOption(Direction.SOUTH)) {
			IPose2D pose = getInOption(Direction.SOUTH).getPose();
			return new Vector2D(pose.getX(), pose.getY() - CROSSING_HALF_LENGTH + LANE_HALF_WIDTH);
		} else {
			// each crossing either has a south or east option
			IPose2D pose = getInOption(Direction.EAST).getPose();
			return new Vector2D(pose.getX() - CROSSING_HALF_LENGTH - LANE_HALF_WIDTH, pose.getY());
		}
	}

	/**
	 * @return the rotation of the segment with respect to the AADC manual's tile orientation
	 */
	public Angle getRotation()
	{
		if (!hasInOption(Direction.SOUTH)) {
			// a north east west T crossing
			return Angle.ANGLE_90;
		}

		switch (getCrossingType(Direction.NORTH)) {
		case EAST_SOUTH_WEST:
			return Angle.ANGLE_90.negate();
		case NORTH_SOUTH_WEST:
			return Angle.ANGLE_180;
		case NORTH_EAST_SOUTH_WEST:
		case NORTH_EAST_SOUTH:
		default:
			return Angle.ZERO;
		}
	}

	public boolean contains(IPose2D pose)
	{
		return contains(pose.getPosition());
	}

	public boolean contains(Vector3D position)
	{
		return getArea().contains(position);
	}

	public Area2D.Float getArea()
	{
		Vector2D position = getPosition();
		double x = position.getX();
		double y = position.getY();
		// try to improve this
		Direction dir = Direction.getDirection(getRotation());
		if (dir == Direction.NORTH || dir == Direction.SOUTH) {
			return new Area2D.Float(x, x + type.width, y, y + type.height);
		}
		return new Area2D.Float(x, x + type.height, y, y + type.width);
	}

	public SegmentLink getClosestLinkToPose(IPose2D pose)
	{
		double maxDistance = Double.MAX_VALUE;
		SegmentLink bestOption = null;
		for (Direction current : Direction.values()) {
			double distance;
			if (hasOutOption(current)) {
				distance = getOutOption(current).getPose().getDistanceTo(pose);
				if (distance < maxDistance) {
					maxDistance = distance;
					bestOption = getOutOption(current);
				}
			}
			if (hasInOption(current)) {
				distance = getInOption(current).getPose().getDistanceTo(pose);
				if (distance < maxDistance) {
					maxDistance = distance;
					bestOption = getInOption(current);
				}
			}
		}
		return bestOption;
	}

	/**
	 * Replaces the connection specified with the replacement
	 * @param source the link to replace (not null)
	 * @param replacement the link with which to replace
	 */
	public void replaceLink(SegmentLink source, SegmentLink replacement)
	{
		// road signs, if added already, have to be merged
		replacement.roadSigns.addAll(source.roadSigns);

		// also save over speed values
		if (replacement.getSpeed() != source.getSpeed()) {
			if (replacement.getSpeed() == IAudiCupMotor.DEFAULT_SPEED) {
				replacement.setSpeed(source.getSpeed());
			} else if (source.getSpeed() != IAudiCupMotor.DEFAULT_SPEED) {
				System.out.println("Connecting segments with different non-default speed: " + replacement.getSpeed() +
								   " vs " + source.getSpeed());
			}
		}

		for (Direction current : Direction.values()) {
			if (source == getOutOption(current)) {
				outOptions[current.ordinal()] = replacement;
				return;
			}
			if (source == getInOption(current)) {
				inOptions[current.ordinal()] = replacement;
				return;
			}
		}
		throw new IllegalArgumentException("Source link not existing! " + source);
	}

	/**
	 * @param facingDirection the direction we are facing when entering the segment
	 * @return the out link of the lane specified by facingDirection
	 */
	public SegmentLink getSameLaneOutOption(Direction facingDirection)
	{
		return getOutOption(facingDirection);
	}

	/**
	 * Returns this or the next segment depending where the passed pose is
	 * @param inLink the link used to enter this segment
	 * @param pose the global pose to check if it is in this or the next segment
	 * @return the segment in which we currently are
	 */
	public Segment getNextOrCurrentSegmentFromPose(SegmentLink inLink, IPose2D pose)
	{
		// set beginLink as origin
		IPose2D beginPose = inLink.getPose();
		double halfLength = ISegment.CROSSING_HALF_LENGTH;

		// check if in current
		// removed poseRelativeToMiddle.getX() >= 0 &&  in comparison to C++ version: we return this if we are before
		// this
		IPose2D middleOfCrossingLocal = new Pose2D(halfLength, ISegment.LANE_HALF_WIDTH);
		IPose2D poseMiddleCrossing = beginPose.applyTo(middleOfCrossingLocal);
		IPose2D poseRelativeToMiddle = poseMiddleCrossing.applyInverseTo(pose);
		if (poseRelativeToMiddle.getX() <= halfLength && poseRelativeToMiddle.getY() >= -halfLength &&
				poseRelativeToMiddle.getY() <= halfLength) {
			return this;
		}

		// this is different to C++ version. How could C++ version unit tests not have failed here?
		// original: 		Angle toPose = poseRelativeToMiddle.getAngle();
		// This must be wrong, we are only interested in the position of the pose relative to the middle of the crossing
		Angle toPose = new Pose2D().getAngleTo(poseRelativeToMiddle);

		Direction dir = Direction.getDirection(toPose.add(beginPose.getAngle()));
		if (hasOutOption(dir)) {
			return getOutOption(dir).getSegmentAfter();
		}
		return this;
	}

	/**
	 * @param outPose pose of the corresponding out link
	 * @return the pose of the in link that is next to the out link's pose
	 */
	public static IPose2D getInPoseFromCorrespondingOutPose(IPose2D outPose)
	{
		Pose2D localInPose = new Pose2D(0, ISegment.LANE_WIDTH, Angle.ANGLE_180);
		return outPose.applyTo(localInPose);
	}

	/**
	 *
	 * @param facingDirection the direction we are facing when entering
	 * @return the type of this crossing when coming from passed inDirection
	 */
	public CrossingType getCrossingType(Direction facingDirection)
	{
		if (!isCrossing()) {
			return null;
		}

		if (getType() == SegmentType.X_CROSSING) {
			return CrossingType.NORTH_EAST_SOUTH_WEST;
		}
		if (getStraightOption(facingDirection) == null) {
			return CrossingType.EAST_SOUTH_WEST;
		}
		if (getLeftOption(facingDirection) != null) {
			return CrossingType.NORTH_SOUTH_WEST;
		}
		return CrossingType.NORTH_EAST_SOUTH;
	}

	public SegmentLink getLeftOption(Direction facingDirection)
	{
		return getOutOption(Direction.getLeft(facingDirection));
	}

	public SegmentLink getRightOption(Direction facingDirection)
	{
		return getOutOption(Direction.getRight(facingDirection));
	}

	public SegmentLink getStraightOption(Direction facingDirection)
	{
		return getSameLaneOutOption(facingDirection);
	}

	/**
	 * @param nextSegment the destination segment
	 * @return the segmentLink that links this segment with the passed segment, null if there is no such link
	 */
	public SegmentLink getLinkConnectingTo(Segment nextSegment)
	{
		for (Direction current : Direction.values()) {
			if (hasOutOption(current)) {
				SegmentLink outOption = getOutOption(current);
				if (outOption.hasSegmentAfter() && outOption.getSegmentAfter().getID() == nextSegment.getID()) {
					return outOption;
				}
			}
		}

		if (hasAttachedOption() && getAttachedOption().getSegmentAfter().getID() == nextSegment.getID()) {
			return getAttachedOption();
		}

		return null;
	}

	@Override
	public String toString()
	{
		StringBuffer result = new StringBuffer();
		result.append("" + id + ": ");
		for (Direction current : Direction.values()) {
			if (hasOutOption(current)) {
				result.append(current + " ");
			}
		}
		return result.toString();
	}
}
