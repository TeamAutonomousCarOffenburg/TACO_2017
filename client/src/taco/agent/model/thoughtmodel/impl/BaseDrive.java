package taco.agent.model.thoughtmodel.impl;

import static hso.autonomy.util.geometry.VectorUtils.to2D;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.logging.DrawingMap;
import taco.agent.model.agentmodel.IUltrasonic;

public abstract class BaseDrive
{
	/** the distance in which obstacles are not save calculated from the front of the car */
	protected double distance;

	/** the length (in m) of the car */
	protected double carLength;

	/** global pose of the car */
	protected IPose2D carPose;

	public BaseDrive(IPose2D carPose, double distance, double carLength)
	{
		this.distance = distance;
		this.carPose = carPose;
		this.carLength = carLength;
	}

	protected abstract void drawDriveArea(DrawingMap drawings);

	/**
	 * @param objectPosition the position in car coordinates
	 * @param distance the distance in m from front of car
	 * @return true if the object is in our drive way
	 */
	public abstract boolean isPositionInWay(Vector2D objectPosition, double distance);

	/**
	 * @param us the ultrasonic sensor for which to check
	 * @param distance the distance in m from front of car
	 * @return the distance of the obstacle if in way, Double.POSITIVE_INFINITY if not in way
	 */
	public double getObstacleDistance(IUltrasonic us, double distance)
	{
		// check position
		Vector2D objectPosition = to2D(us.getObjectPosition());
		if (!isPositionInWay(objectPosition, distance)) {
			return Double.POSITIVE_INFINITY;
		}

		// check distance
		return us.getDistance();
	}

	protected boolean checkDistance(Vector2D objectPosition, double distance)
	{
		return objectPosition.distance(new Vector2D(carLength, 0)) < distance;
	}
}
