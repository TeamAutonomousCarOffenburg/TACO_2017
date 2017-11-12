package taco.agent.model.worldmodel.street;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;

/**
 * Representation of what Audi calls a sector, i.e. a block of drive instructions
 */
public class Sector
{
	private static final IPose2D OFFSET = new Pose2D(-0.5, 0, Angle.ZERO);

	/** the ID of the segment of the underlying map that is the start segment of this sector */
	private int segmentIndex;

	private IPose2D startPose;

	/**
	 * @param segmentIndex the ID of the segment of the underlying map that is the start segment of this sector
	 * @param startPose the pose of the car where to start the sector
	 */
	public Sector(int segmentIndex, IPose2D startPose)
	{
		this.segmentIndex = segmentIndex;
		this.startPose = startPose.applyTo(OFFSET);
	}

	public int getSegmentIndex()
	{
		return segmentIndex;
	}

	public void setSegmentIndex(int segmentIndex)
	{
		this.segmentIndex = segmentIndex;
	}

	public IPose2D getStartPose()
	{
		return startPose;
	}
}
