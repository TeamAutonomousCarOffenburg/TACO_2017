package taco.agent.model.worldmodel.street;

import static taco.agent.model.agentmodel.IAudiCupMotor.DEFAULT_SPEED;
import static taco.agent.model.worldmodel.street.ISegment.CROSSING_HALF_LENGTH;
import static taco.agent.model.worldmodel.street.ISegment.CROSSING_LENGTH;
import static taco.agent.model.worldmodel.street.ISegment.LANE_HALF_WIDTH;
import static taco.agent.model.worldmodel.street.ISegment.LANE_WIDTH;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.agent.model.worldmodel.signdetection.RoadSign;

public class SegmentLink
{
	/** poses of the out links of a crossing relative to the south in link */
	static IPose2D[] OUT_POSES = {
			new Pose2D(CROSSING_LENGTH, 0, Angle.ZERO), // North
			new Pose2D(CROSSING_HALF_LENGTH - LANE_HALF_WIDTH, -CROSSING_HALF_LENGTH + LANE_HALF_WIDTH,
					Angle.ANGLE_90.negate()),			// East
			new Pose2D(0, LANE_WIDTH, Angle.ANGLE_180), // South
			new Pose2D(CROSSING_HALF_LENGTH + LANE_HALF_WIDTH, CROSSING_HALF_LENGTH + LANE_HALF_WIDTH,
					Angle.ANGLE_90), // West
	};

	/** poses of the out links of s curves relative to the south in link */
	static IPose2D[] S_POSES = {
			new Pose2D(3, 2, Angle.ZERO),  // left
			new Pose2D(3, -2, Angle.ZERO), // right
	};

	/** global pose of the link */
	private IPose2D pose;

	/** segment this link links from */
	Segment segmentBefore;

	/** segment this link links to */
	Segment segmentAfter;

	/** a list of road signs attached to this segmentLink (semantically to the in link) */
	List<RoadSign> roadSigns;

	/** the speed limit for the segment before this out link */
	private double speed;

	private boolean hasStopLine;

	SegmentLink(IPose2D pose, Segment segmentBefore, Segment segmentAfter)
	{
		this(pose, segmentBefore, segmentAfter, IAudiCupMotor.DEFAULT_SPEED);
	}

	SegmentLink(IPose2D pose, Segment segmentBefore, Segment segmentAfter, double speed)
	{
		this.pose = pose;
		this.segmentBefore = segmentBefore;
		this.segmentAfter = segmentAfter;
		this.speed = speed;
		roadSigns = new ArrayList<>();
		hasStopLine = false;
	}

	public List<RoadSign> getRoadSigns()
	{
		return roadSigns;
	}

	void addRoadSigns(List<RoadSign> roadSigns)
	{
		this.roadSigns.addAll(roadSigns);
	}

	public void addRoadSign(RoadSign roadSign)
	{
		this.roadSigns.add(roadSign);
	}

	void setPose(IPose2D pose)
	{
		this.pose = pose;
	}

	public IPose2D getPose()
	{
		return pose;
	}

	public double getSpeed()
	{
		return speed;
	}

	public void setSpeed(double speed)
	{
		this.speed = speed;
	}

	public Direction getDirection()
	{
		return getDirection(pose);
	}

	public static Direction getDirection(IPose2D pose)
	{
		return Direction.getDirection(pose.getAngle());
	}

	public Segment getSegmentBefore()
	{
		return segmentBefore;
	}

	public Segment getSegmentAfter()
	{
		return segmentAfter;
	}

	boolean hasSegmentBefore()
	{
		if (segmentBefore != null) {
			return true;
		}

		return false;
	}

	public boolean hasSegmentAfter()
	{
		return segmentAfter != null;
	}

	/**
	 * @param pose the pose to check
	 * @return true if the passed pose is in front of this pose
	 */
	public boolean hasPassedLink(IPose2D pose)
	{
		Angle angleToPose = this.pose.getAngleTo(pose);
		return Math.abs(angleToPose.degrees()) < 90;
	}

	void setSegmentAfter(Segment segment)
	{
		segmentAfter = segment;
	}

	void setSegmentBefore(Segment segment)
	{
		segmentBefore = segment;
	}

	/**
	 * @return the in link that is on the other lane at the same end of the street
	 */
	public SegmentLink getCorrespondingInLink()
	{
		return getSegmentBefore().getInOption(getDirection());
	}

	/**
	 * @return the out link that is on the other lane at the same end of the street
	 */
	public SegmentLink getCorrespondingOutLink()
	{
		return getSegmentAfter().getOutOption(Direction.getOppositeDirection(getDirection()));
	}

	/**
	 * @return the out link that is on the same lane at the straight option
	 */
	public SegmentLink getSameLaneOutOption()
	{
		return getSegmentAfter().getSameLaneOutOption(this.getDirection());
	}

	// create methods
	public NonCrossingSegment appendStraightSegment()
	{
		return appendStraightSegment(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendStraightSegment(double speedOur, double speedOther)
	{
		return appendStraightSegment(1, speedOur, speedOther);
	}

	public NonCrossingSegment appendStraightSegment(double length, double speedOur, double speedOther)
	{
		return appendNonCrossing(length, Angle.ZERO, SegmentType.STRAIGHT, speedOur, speedOther);
	}

	public NonCrossingSegment appendStraightSegmentWithCrosswalk()
	{
		return appendStraightSegmentWithCrosswalk(1.0, DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendStraightSegmentWithCrosswalk(double length, double speedOur, double speedOther)
	{
		return appendNonCrossing(length, Angle.ZERO, SegmentType.STRAIGHT_WITH_CROSSWALK, speedOur, speedOther);
	}

	public NonCrossingSegment appendCurveSmallLeft()
	{
		return appendCurveSmallLeft(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendCurveSmallLeft(double speedOur, double speedOther)
	{
		return appendNonCrossing(3 * Math.PI / 4, Angle.ANGLE_90, SegmentType.CURVE_SMALL, speedOur, speedOther);
	}

	public NonCrossingSegment appendCurveSmallRight()
	{
		return appendCurveSmallRight(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendCurveSmallRight(double speedOur, double speedOther)
	{
		return appendNonCrossing(
				3 * Math.PI / 4, Angle.ANGLE_90.negate(), SegmentType.CURVE_SMALL, speedOur, speedOther);
	}

	public NonCrossingSegment appendCurveBigLeft()
	{
		return appendCurveBigLeft(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendCurveBigLeft(double speedOur, double speedOther)
	{
		return appendNonCrossing(5 * Math.PI / 4, Angle.ANGLE_90, SegmentType.CURVE_BIG, speedOur, speedOther);
	}

	public NonCrossingSegment appendCurveBigRight()
	{
		return appendCurveBigRight(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendCurveBigRight(double speedOur, double speedOther)
	{
		return appendNonCrossing(5 * Math.PI / 4, Angle.ANGLE_90.negate(), SegmentType.CURVE_BIG, speedOur, speedOther);
	}

	public NonCrossingSegment appendSCurveBottom()
	{
		return appendSCurveBottom(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendSCurveBottom(double speedOur, double speedOther)
	{
		return appendSCrossing(S_POSES[0], SegmentType.S_CURVE_BOTTOM, speedOur, speedOther);
	}

	public NonCrossingSegment appendSCurveTop()
	{
		return appendSCurveTop(DEFAULT_SPEED, DEFAULT_SPEED);
	}

	public NonCrossingSegment appendSCurveTop(double speedOur, double speedOther)
	{
		return appendSCrossing(S_POSES[1], SegmentType.S_CURVE_TOP, speedOur, speedOther);
	}

	/**
	 *
	 * @param deltaPose the relative pose of the out link same lane
	 * @return the new segment created
	 */
	private NonCrossingSegment appendSCrossing(IPose2D deltaPose, SegmentType type, double speedOur, double speedOther)
	{
		// Create a new segment
		// 2 * 1/4 circumference(1.5)  - 1
		NonCrossingSegment newSegment = new NonCrossingSegment(3.7, Angle.ZERO, type);

		// Wire up new segment with this link
		connectToSegment(newSegment, speedOther);

		// create straight out option
		IPose2D outPose = pose.applyTo(deltaPose);
		createOption(newSegment, getDirection(outPose), outPose, speedOur);

		return newSegment;
	}

	/**
	 *
	 * @param length the length of the segment middle line
	 * @param bendingAngle the angle the segment bends (positive is to left)
	 * @return the new segment created
	 */
	private NonCrossingSegment appendNonCrossing(
			double length, Angle bendingAngle, SegmentType type, double speedOur, double speedOther)
	{
		// Create a new segment
		NonCrossingSegment newSegment = new NonCrossingSegment(length, bendingAngle, type);

		// Wire up new segment with this link
		connectToSegment(newSegment, speedOther);

		// create straight out option
		IPose2D outPose = NonCrossingSegment.getCurveOutPose(pose, length, bendingAngle);
		createOption(newSegment, getDirection(outPose), outPose, speedOur);

		return newSegment;
	}

	private void createOption(Segment newSegment, Direction direction, IPose2D outPose, double speed)
	{
		SegmentLink outLink = new SegmentLink(outPose, newSegment, null, speed);
		newSegment.setOutOption(direction, outLink);

		// create straight in option
		IPose2D inPose = Segment.getInPoseFromCorrespondingOutPose(outPose);
		SegmentLink inLink = new SegmentLink(inPose, null, newSegment);
		newSegment.setInOption(direction, inLink);
	}

	/**
	 * @param newSegment the segment to link to
	 * @return the direction that is set on the new segment
	 */
	private Direction connectToSegment(Segment newSegment, double speed)
	{
		Direction direction = getDirection();
		Direction oppositeDirection = Direction.getOppositeDirection(direction);
		newSegment.setInOption(oppositeDirection, this);
		setSegmentAfter(newSegment);
		SegmentLink otherInOption = getSegmentBefore().getInOption(direction);
		// set the speed of the new segment's out option (other lane)
		otherInOption.setSpeed(speed);
		newSegment.setOutOption(oppositeDirection, otherInOption);
		otherInOption.setSegmentBefore(newSegment);
		return oppositeDirection;
	}

	// Crossings
	public Segment appendXCrossing()
	{
		return appendXCrossing(DEFAULT_SPEED);
	}

	public Segment appendXCrossing(double speed)
	{
		Direction[] dirs = {Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST};
		return appendCrossing(dirs, SegmentType.X_CROSSING, speed);
	}

	public Segment appendTCrossingLS()
	{
		return appendTCrossingLS(DEFAULT_SPEED);
	}

	public Segment appendTCrossingLS(double speed)
	{
		Direction[] dirs = {Direction.NORTH, Direction.SOUTH, Direction.WEST};
		return appendCrossing(dirs, SegmentType.T_CROSSING, speed);
	}

	public Segment appendTCrossingLR()
	{
		return appendTCrossingLR(DEFAULT_SPEED);
	}

	public Segment appendTCrossingLR(double speed)
	{
		Direction[] dirs = {Direction.EAST, Direction.SOUTH, Direction.WEST};
		return appendCrossing(dirs, SegmentType.T_CROSSING, speed);
	}

	public Segment appendTCrossingSR()
	{
		return appendTCrossingSR(DEFAULT_SPEED);
	}

	public Segment appendTCrossingSR(double speed)
	{
		Direction[] dirs = {Direction.NORTH, Direction.EAST, Direction.SOUTH};
		return appendCrossing(dirs, SegmentType.T_CROSSING, speed);
	}

	/**
	 * @param directions relative directions so this links pose direction.
	 * (straight = North, right = East, back = South, left = West);
	 * @return the new segment
	 */
	private Segment appendCrossing(Direction[] directions, SegmentType type, double speed)
	{
		// Create a new segment
		Segment newSegment = new Segment(type);

		// Wire up new segment with preceding one
		Direction dirAtNew = connectToSegment(newSegment, speed);

		// create links
		Direction ourDirection = getDirection();
		for (Direction currentDir : directions) {
			Direction absoluteDir = Direction.getAbsoluteFromRelativeDirection(ourDirection, currentDir);
			if (absoluteDir != dirAtNew) {
				IPose2D outPose = pose.applyTo(OUT_POSES[currentDir.ordinal()]);
				createOption(newSegment, absoluteDir, outPose, speed);
			}
		}

		return newSegment;
	}

	/**
	 * Connects this link (must be an out link) with the passed link (must be an inLink). Assumes
	 * that the two poses are close with respect to position and angle. Throws an IllegalArgumentException if this is
	 * not the case.
	 * @param connectTo the in link to connect to
	 */
	public void connectToInLink(SegmentLink connectTo)
	{
		double length = pose.getDistanceTo(connectTo.getPose());
		if (length > 0.1) {
			throw new IllegalArgumentException("Pose not close on connection: " + length + " this: " + toString() +
											   " other: " + connectTo.toString());
		}
		Angle bendingAngle = pose.getDeltaAngle(connectTo.getPose());
		if (Math.abs(bendingAngle.degrees()) > 1.0) {
			throw new IllegalArgumentException("Pose angles not fitting on connection: " + bendingAngle.degrees() +
											   " this: " + toString() + " other: " + connectTo.toString());
		}

		// same spot, so merge the in with the out and the out with the in link
		this.segmentAfter = connectTo.segmentAfter;
		connectTo.getSegmentAfter().replaceLink(connectTo, this);

		SegmentLink inLink = getCorrespondingInLink();
		SegmentLink outLink = connectTo.getCorrespondingOutLink();
		inLink.segmentBefore = outLink.segmentBefore;
		outLink.getSegmentBefore().replaceLink(outLink, inLink);
	}

	@Override
	public String toString()
	{
		StringBuffer result = new StringBuffer();
		result.append((segmentBefore == null) ? "None - " : segmentBefore.toString() + " - ");
		result.append(pose.toString());
		result.append((segmentAfter == null) ? " - None" : " - " + segmentAfter.toString());
		return result.toString();
	}

	public boolean hasStopLine()
	{
		return hasStopLine;
	}

	public void addStopLine()
	{
		hasStopLine = true;
	}
}
