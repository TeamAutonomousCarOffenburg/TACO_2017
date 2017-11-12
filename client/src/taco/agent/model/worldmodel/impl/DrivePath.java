package taco.agent.model.worldmodel.impl;

import java.util.ArrayList;
import java.util.List;

import hso.autonomy.util.geometry.IPose2D;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;

public class DrivePath
{
	/** a list of global poses defining the planned trajectory of this car an the maneuver that leads to it */
	private List<DrivePoint> drivePath;

	public DrivePath()
	{
		drivePath = new ArrayList<>();
	}

	public DrivePath(SegmentLink[] links, IPose2D[] poses, DriveInstruction[] instructions)
	{
		this();
		assert(poses.length == instructions.length)
			: "Invalid amount of array elements: " + poses.length + " instructions: " + instructions.length;

		int i = 0;
		for (IPose2D pose : poses) {
			add(links[i], pose, new Maneuver(instructions[i], 0, i));
			i++;
		}
	}

	public void add(SegmentLink goalLink, IPose2D pose, Maneuver maneuver)
	{
		drivePath.add(new DrivePoint(goalLink, pose, maneuver));
	}

	public void add(SegmentLink goalLink, Maneuver maneuver)
	{
		drivePath.add(new DrivePoint(goalLink, maneuver));
	}

	public DrivePoint lastOccurrenceOf(Segment toFind)
	{
		return drivePath.stream()
				.filter(point
						-> point.getGoalLink().getSegmentBefore() != null &&
								   point.getGoalLink().getSegmentBefore().getID() == toFind.getID())
				.reduce((first, second) -> second)
				.orElse(null);
	}

	public List<DrivePoint> getDrivePath()
	{
		return drivePath;
	}
}
