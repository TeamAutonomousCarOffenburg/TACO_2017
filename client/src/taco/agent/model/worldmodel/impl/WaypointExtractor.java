package taco.agent.model.worldmodel.impl;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.RuntimeSegment;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;

/**
 * Extracts waypoints from the current segment/map and drive instructions
 */
public class WaypointExtractor
{
	static final Pose2D TURN_RIGHT_OFFSET_BEFORE = new Pose2D(-0.4, 0.13, Angle.deg(0));

	static final Pose2D TURN_RIGHT_OFFSET_AFTER = new Pose2D(0.2, 0.1, Angle.deg(0));

	public static DrivePath extractPath(DriveInstructionManager driveInstructionManager, RuntimeSegment currentSegment)
	{
		RuntimeSegment runningSegment = new RuntimeSegment(currentSegment);
		int instructionIndex = driveInstructionManager.getCurrentInstructionIndex();
		DrivePath result = new DrivePath();

		// add initial pose to waypoints
		DriveInstruction previousInstruction = DriveInstruction.FOLLOW_LANE;
		if (runningSegment.isCrossing() && instructionIndex > 0) {
			previousInstruction = driveInstructionManager.getInstruction(instructionIndex - 1);
		}
		Maneuver firstManeuver = driveInstructionManager.getManeuver(instructionIndex);
		runningSegment.update(firstManeuver, firstManeuver);
		result.add(runningSegment.getInLink(),
				new Maneuver(previousInstruction, firstManeuver.getSector(), instructionIndex));

		// Run through all known segments and plan waypoints along them as long as we have driving instructions left
		while (instructionIndex < driveInstructionManager.getNumberOfInstructions()) {
			Maneuver previousManeuver = driveInstructionManager.getManeuver(instructionIndex - 1);
			Maneuver currentManeuver = driveInstructionManager.getManeuver(instructionIndex);
			DriveInstruction currentInstruction = currentManeuver.getDriveInstruction();
			runningSegment.update(previousManeuver, currentManeuver);
			SegmentLink nextLink = runningSegment.getIntendedOption();
			if (runningSegment.consumesDriveInstruction()) {
				// right turns need a bigger curve radius than the street provides
				if (currentInstruction == DriveInstruction.RIGHT && !result.getDrivePath().isEmpty()) {
					if (result.getDrivePath().size() > 1) {
						DrivePoint last = result.getDrivePath().remove(result.getDrivePath().size() - 1);
						result.add(last.getGoalLink(), last.getGoalLink().getPose().applyTo(TURN_RIGHT_OFFSET_BEFORE),
								last.getManeuver());
					}
					result.add(nextLink, nextLink.getPose().applyTo(TURN_RIGHT_OFFSET_AFTER), currentManeuver);

				} else {
					// all others we aim at the link position
					result.add(nextLink, currentManeuver);
				}
				instructionIndex++;

			} else {
				// straight or curve
				result.add(nextLink,
						new Maneuver(DriveInstruction.FOLLOW_LANE, currentManeuver.getSector(), instructionIndex));
			}

			Segment nextSegment = nextLink.getSegmentAfter();
			if (nextSegment == null) {
				break;
			}

			DrivePoint lastExisting = result.lastOccurrenceOf(nextSegment);
			if (lastExisting != null && lastExisting.getInstructionIndex() >= instructionIndex) {
				if (currentInstruction != DriveInstruction.STRAIGHT_FOREVER) {
					System.out.println(
							"Cycle detected when calculating drive instructions! Drive instructions do not match map or current segment!");
				}
				return result;
			}

			runningSegment.switchToSegment(nextSegment, nextLink.getPose().getAngle());
		}
		return result;
	}
}
