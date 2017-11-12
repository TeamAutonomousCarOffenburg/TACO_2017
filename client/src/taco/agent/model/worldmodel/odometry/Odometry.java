package taco.agent.model.worldmodel.odometry;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;

/**
 * Base class for all odometry classes that are able to keep track of our pose.
 */
public class Odometry
{
	/** the current pose estimation */
	protected IPose2D pose;

	/** the start pose to start with */
	protected IPose2D startPose;

	public Odometry(IPose2D startPose)
	{
		this.startPose = startPose;
		pose = startPose;
	}

	public IPose2D getPose()
	{
		return pose;
	}

	public void init()
	{
		pose = startPose;
	}

	public void setPose(IPose2D pose)
	{
		this.pose = pose;
	}

	public void setPosition(Vector2D position)
	{
		this.pose = new Pose2D(position, pose.getAngle());
	}
}
