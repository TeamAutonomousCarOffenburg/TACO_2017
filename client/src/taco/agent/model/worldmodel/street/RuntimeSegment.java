package taco.agent.model.worldmodel.street;

import java.util.List;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.impl.Maneuver;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.util.SignType;
import taco.util.rightofway.CrossingType;

/**
 * Class representing a patch of street.
 */
public class RuntimeSegment
{
	private Segment decoratee;

	protected SegmentLink inLink;

	protected SegmentLink intendedOption;

	protected boolean consumesDriveInstruction;

	public RuntimeSegment(Segment decoratee, Angle globalAngle)
	{
		this.decoratee = decoratee;
		Direction direction = Direction.getOppositeDirection(Direction.getDirection(globalAngle));
		inLink = decoratee.getInOption(direction);
		if (inLink == null) {
			inLink = decoratee.getAlternativeInOption(direction);
		}
		intendedOption = inLink.getSameLaneOutOption();
		consumesDriveInstruction = false;
	}

	public RuntimeSegment(RuntimeSegment currentSegment)
	{
		this.decoratee = currentSegment.decoratee;
		this.inLink = currentSegment.inLink;
		this.intendedOption = currentSegment.intendedOption;
		this.consumesDriveInstruction = currentSegment.consumesDriveInstruction;
	}

	public int getID()
	{
		return decoratee.getID();
	}

	SegmentLink getLeftOption()
	{
		return decoratee.getLeftOption(inLink.getDirection());
	}

	SegmentLink getStraightOption()
	{
		return decoratee.getStraightOption(inLink.getDirection());
	}

	SegmentLink getBackOption()
	{
		return inLink.getCorrespondingOutLink();
	}

	SegmentLink getRightOption()
	{
		return decoratee.getRightOption(inLink.getDirection());
	}

	public SegmentLink getIntendedOption()
	{
		return intendedOption;
	}

	public SegmentLink getInLink()
	{
		return inLink;
	}

	boolean hasLeftOption()
	{
		return getLeftOption() != null;
	}

	boolean hasRightOption()
	{
		return getRightOption() != null;
	}

	/**
	 * @return true if there
	 */
	boolean hasOppositeSideOutOption()
	{
		return decoratee.hasOutOption(Direction.getOppositeDirection(inLink.getDirection()));
	}

	/**
	 * @return the type of crossing we have, null if not a crossing
	 */
	public CrossingType getCrossingType()
	{
		return decoratee.getCrossingType(inLink.getDirection());
	}

	public boolean consumesDriveInstruction()
	{
		return consumesDriveInstruction;
	}

	public List<RoadSign> getRoadSigns(Direction direction)
	{
		return decoratee.getRoadSigns(direction);
	}

	public void addRoadSigns(List<RoadSign> roadSigns, Direction direction)
	{
		decoratee.addRoadSigns(roadSigns, direction);
	}

	public boolean isCrossing()
	{
		return decoratee.isCrossing();
	}

	public boolean isStraight()
	{
		return decoratee.isStraight();
	}

	boolean hasIntendedOption()
	{
		return intendedOption != null;
	}

	boolean isLeftOptionAllowed()
	{
		SegmentLink option = getLeftOption();
		if (option == null) {
			return false;
		}
		for (RoadSign sign : inLink.getRoadSigns()) {
			if (sign.getSignType() == SignType.AHEAD_ONLY) {
				return false;
			}
		}
		return true;
	}

	boolean isStraightOptionAllowed()
	{
		SegmentLink option = getStraightOption();
		if (option == null) {
			return false;
		}
		// Currently no sign prevents us from taking this option
		return true;
	}

	boolean isRightOptionAllowed()
	{
		SegmentLink option = getRightOption();
		if (option == null) {
			return false;
		}
		for (RoadSign sign : inLink.getRoadSigns()) {
			if (sign.getSignType() == SignType.AHEAD_ONLY) {
				return false;
			}
		}
		return true;
	}

	public SegmentLink getClosestLinkToPose(IPose2D pose)
	{
		return decoratee.getClosestLinkToPose(pose);
	}

	public void switchToSegment(Segment nextSegment, Angle carAngle)
	{
		// find link that connects the two segments
		inLink = decoratee.getLinkConnectingTo(nextSegment);
		if (inLink == null) {
			// for parking spaces this might be the case
			inLink = nextSegment.getLinkConnectingTo(decoratee);
			if (inLink == null) {
				// we switch to a segment that is not linked to this
				// take direction from next to this as indicator
				Angle angle = Angle.to(decoratee.getPosition().subtract(nextSegment.getPosition()));
				Direction direction = Direction.getDirection(angle);
				inLink = nextSegment.getInOption(direction);
				if (inLink == null) {
					// we reached this segment not through a valid lane
					inLink = nextSegment.getAlternativeInOption(direction);
				}
			}
		} else {
			if (Math.abs(inLink.getPose().getAngle().subtract(carAngle).degrees()) > 90) {
				// we are driving backward
				Direction direction = Direction.getOppositeDirection(Direction.getDirection(carAngle));
				inLink = nextSegment.getInOption(direction);
				if (inLink == null) {
					inLink = nextSegment.getAlternativeInOption(direction);
				}
			}
		}
		this.decoratee = nextSegment;
	}

	/**
	 * Updates this segment with information on the next drive instruction.
	 * @param previousManeuver the maneuver before the currentManeuver
	 * @param currentManeuver the maneuver that is planned next
	 */
	public DriveInstruction update(Maneuver previousManeuver, Maneuver currentManeuver)
	{
		DriveInstruction previousInstruction = previousManeuver.getDriveInstruction();
		DriveInstruction driveInstruction = currentManeuver.getDriveInstruction();
		DriveInstruction result = driveInstruction;
		int pullOutParkingID = 1;
		if (previousInstruction == DriveInstruction.CROSS_PARKING) {
			pullOutParkingID = previousManeuver.getInstructionSubID();
		}
		consumesDriveInstruction = false;

		switch (driveInstruction) {
		case CROSS_PARKING:
			if (decoratee.hasAttachedOption()) {
				Segment attached = decoratee.getAttachedOption().getSegmentAfter();
				if (attached instanceof ParkingSegment) {
					intendedOption = decoratee.getAttachedOption();
					consumesDriveInstruction = true;
					return DriveInstruction.CROSS_PARKING;
				}
			}
			break;

		case PULL_OUT_RIGHT:
			if (decoratee instanceof ParkingSegment) {
				// should always be the case
				if (pullOutParkingID == 1) {
					// if this is the first instruction, we do not know the parkingID we stand on
					pullOutParkingID = ((ParkingSegment) decoratee).getFirstID();
				}
				intendedOption = ((ParkingSegment) decoratee).getOutOptionByParkingID(pullOutParkingID);
			} else {
				intendedOption = getStraightOption();
				if (intendedOption == null) {
					intendedOption = getRightOption();
				}
			}
			consumesDriveInstruction = true;
			return DriveInstruction.PULL_OUT_RIGHT;

		case PULL_OUT_LEFT:
			if (decoratee instanceof ParkingSegment) {
				// should always be the case
				if (pullOutParkingID == 1) {
					pullOutParkingID = ((ParkingSegment) decoratee).getFirstID();
				}
				intendedOption = ((ParkingSegment) decoratee).getOutOptionByParkingID(pullOutParkingID);
			} else {
				intendedOption = getBackOption();
				if (intendedOption == null) {
					intendedOption = getStraightOption();
				}
			}
			consumesDriveInstruction = true;
			return DriveInstruction.PULL_OUT_LEFT;
		}

		if (decoratee.isCrossing()) {
			boolean beforeIsParking = inLink.hasSegmentBefore() && inLink.getSegmentBefore() instanceof ParkingSegment;
			if (previousInstruction == DriveInstruction.PULL_OUT_LEFT && beforeIsParking) {
				intendedOption = getLeftOption();
			} else if (previousInstruction == DriveInstruction.PULL_OUT_RIGHT && beforeIsParking) {
				intendedOption = getRightOption();
			} else if ((driveInstruction == DriveInstruction.LEFT) && isLeftOptionAllowed()) {
				intendedOption = getLeftOption();
				consumesDriveInstruction = true;
			} else if (driveInstruction == DriveInstruction.STRAIGHT && isStraightOptionAllowed()) {
				intendedOption = getStraightOption();
				consumesDriveInstruction = true;
			} else if (driveInstruction == DriveInstruction.STRAIGHT_FOREVER && isStraightOptionAllowed()) {
				intendedOption = getStraightOption();
			} else if ((driveInstruction == DriveInstruction.RIGHT) && isRightOptionAllowed()) {
				intendedOption = getRightOption();
				consumesDriveInstruction = true;
			} else {
				// We have no matching instruction for this crossing, decide for the first best option
				intendedOption = getStraightOption();
				result = DriveInstruction.STRAIGHT;
				if (intendedOption == null) {
					intendedOption = getRightOption();
					result = DriveInstruction.RIGHT;
				}
				if (intendedOption == null) {
					intendedOption = getLeftOption();
					result = DriveInstruction.LEFT;
				}
			}
		} else {
			// curve or straight
			if (!hasOppositeSideOutOption()) {
				// this only happens if we are pulling out a parking space or driving crazy
				if (previousInstruction == DriveInstruction.PULL_OUT_LEFT) {
					intendedOption = getLeftOption();
				} else if (previousInstruction == DriveInstruction.PULL_OUT_RIGHT) {
					intendedOption = getRightOption();
				} else {
					// driving crazy, take any
					intendedOption = getStraightOption();
				}
			} else {
				intendedOption = getStraightOption();
			}
			result = DriveInstruction.FOLLOW_LANE;
		}
		return result;
	}

	public Segment getSegment()
	{
		return decoratee;
	}

	@Override
	public String toString()
	{
		return "RuntimeSegment [decoratee=" + decoratee + ", inLink=" + inLink + ", intendedOption=" + intendedOption +
				", consumesDriveInstruction=" + consumesDriveInstruction + "]";
	}
}
