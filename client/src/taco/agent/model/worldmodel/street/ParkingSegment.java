package taco.agent.model.worldmodel.street;

import static taco.agent.model.worldmodel.street.ISegment.CROSSING_HALF_LENGTH;
import static taco.agent.model.worldmodel.street.ISegment.LANE_HALF_WIDTH;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;

/**
 * A segment with parking slots
 */
public class ParkingSegment extends Segment
{
	/** id of the first slot. We assume they are ascending */
	private int firstID;

	private boolean[] occupied;

	/**
	 * @param type horizontal or vertical
	 * @param firstID the lowest id of the four parking slots
	 * @param parentLink in link of the parent segment's closer lane
	 * @param distance the distance of the first spot with respect to the parentLink (axis parallel)
	 */
	public ParkingSegment(
			StreetMap map, SegmentType type, int firstID, SegmentLink parentLink, double distance, boolean[] occupied)
	{
		super(type);
		this.firstID = firstID;
		this.occupied = occupied;

		double step = 0.5;
		if (type == SegmentType.PARKING_SPACE_HORIZONTAL) {
			step = 0.8;
		}

		// create the links
		double y = -CROSSING_HALF_LENGTH + LANE_HALF_WIDTH + 0.01;
		for (int i = 0; i < 4; i++) {
			IPose2D pose = parentLink.getPose().applyTo(new Pose2D(distance + i * step, y, Angle.ANGLE_90.negate()));
			Segment segment = map.getSegmentContaining(pose.getPosition());
			inOptions[i] = new SegmentLink(pose, segment, this);
			IPose2D outPose = new Pose2D(pose.getX(), pose.getY(), pose.getAngle().add(Angle.ANGLE_180));
			outOptions[i] = new SegmentLink(outPose, this, segment);
		}
	}

	public int getFirstID()
	{
		return firstID;
	}

	/**
	 * @return the position of this segment as the south-east corner
	 */
	@Override
	public Vector2D getPosition()
	{
		IPose2D pose = inOptions[0].getPose();
		if (type == SegmentType.PARKING_SPACE_VERTICAL) {
			switch (Direction.getDirection(getRotation())) {
			case EAST:
				pose = pose.applyTo(new Pose2D(0, -0.25));
				break;
			case SOUTH:
				pose = pose.applyTo(new Pose2D(1, -0.25));
				break;
			case WEST:
				pose = pose.applyTo(new Pose2D(1, 1.75));
				break;
			case NORTH:
				pose = pose.applyTo(new Pose2D(0, 1.75));
				break;
			}
		} else {
			switch (Direction.getDirection(getRotation())) {
			case NORTH:
				pose = pose.applyTo(new Pose2D(0, 3.75));
				break;
			case EAST:
				pose = pose.applyTo(new Pose2D(0, -0.25));
				break;
			case SOUTH:
				pose = pose.applyTo(new Pose2D(1, -0.25));
				break;
			case WEST:
				pose = pose.applyTo(new Pose2D(1, 3.75));
				break;
			}
		}
		return new Vector2D(pose.getX(), pose.getY());
	}

	/**
	 * @return the rotation of the segment with respect to the AADC manuals tile orientation
	 */
	@Override
	public Angle getRotation()
	{
		IPose2D pose = inOptions[0].getPose();
		return pose.getAngle().subtract(Angle.ANGLE_90);
	}

	public SegmentLink getInOptionByParkingID(int id)
	{
		int index = id - firstID;
		if (index < 0 || index > inOptions.length) {
			System.err.println("Invalid parking id to get in option: " + id + ". Parkings start from: " + firstID);
			index = 0;
		}
		return inOptions[index];
	}

	public SegmentLink getOutOptionByParkingID(int id)
	{
		int index = id - firstID;
		if (index < 0 || index > outOptions.length) {
			System.err.println("Invalid parking id specified: " + id + ". Parking spaces start from: " + firstID);
			index = 0;
		}
		return outOptions[index];
	}

	public SegmentLink getOutOptionByRelativeParkingID(int id)
	{
		if (id < 0 || id > outOptions.length) {
			System.err.println("Invalid relative parking id specified: " + id + ". Have ids 0-" + outOptions.length);
			id = 0;
		}
		return outOptions[id];
	}

	/**
	 * @param id the relative id of the parking slot (0-3)
	 * @return true if the specified parking slot is occupied
	 */
	public boolean isOccupied(int id)
	{
		return occupied[id];
	}
}
