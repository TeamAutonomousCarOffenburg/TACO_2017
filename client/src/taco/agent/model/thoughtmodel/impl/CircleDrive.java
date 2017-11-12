package taco.agent.model.thoughtmodel.impl;

import java.awt.Color;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Arc2D;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.logging.DrawingMap;

public class CircleDrive extends BaseDrive
{
	private Vector2D circleCenter;
	private double innerRadius;
	private double outerRadius;

	public CircleDrive(IPose2D carPose, double distance, double carLength, Vector2D circleCenter, double innerRadius,
			double outerRadius)
	{
		super(carPose, distance, carLength);
		this.circleCenter = circleCenter;
		this.innerRadius = innerRadius;
		this.outerRadius = outerRadius;
	}

	@Override
	protected void drawDriveArea(DrawingMap drawings)
	{
		Vector2D globalCenter = carPose.applyTo(circleCenter);
		double deltaAngle = Math.asin((distance + 0.6) / (outerRadius * 2)) * 2;
		Angle startAngle = carPose.getAngle().subtract(Angle.ANGLE_90);
		if (circleCenter.getY() < 0) {
			startAngle = startAngle.add(Angle.ANGLE_180).subtract(Angle.rad(deltaAngle));
		}
		Angle endAngle = startAngle.add(Angle.rad(deltaAngle));

		drawings.draw("ObstacleInner", Color.YELLOW,
				new Arc2D(globalCenter.getX(), globalCenter.getY(), innerRadius, startAngle, endAngle));
		drawings.draw("ObstacleOuter", Color.YELLOW,
				new Arc2D(globalCenter.getX(), globalCenter.getY(), outerRadius, startAngle, endAngle));
	}

	/**
	 * @param objectPosition the position in car coordinates
	 * @return true if the object is in our drive way
	 */
	@Override
	public boolean isPositionInWay(Vector2D objectPosition, double distance)
	{
		// check position
		double distanceCenter = objectPosition.distance(circleCenter);
		if (distanceCenter > outerRadius || distanceCenter < innerRadius) {
			return false;
		}
		return checkDistance(objectPosition, distance);
	}
}
