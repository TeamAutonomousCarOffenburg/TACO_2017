package taco.agent.model.thoughtmodel.impl;

import java.awt.Color;

import org.apache.commons.math3.geometry.euclidean.twod.SubLine;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.logging.DrawingMap;

public class StraightDrive extends BaseDrive
{
	private double carWidth;

	public StraightDrive(IPose2D carPose, double distance, double carLength, double carWidth)
	{
		super(carPose, distance, carLength);
		this.carWidth = carWidth;
	}

	/**
	 * @param objectPosition the position in car coordinates
	 * @return true if the object is in our drive way
	 */
	@Override
	public boolean isPositionInWay(Vector2D objectPosition, double distance)
	{
		// check position
		double objectY = objectPosition.getY();
		if (objectY < -carWidth * 0.5 || objectY > carWidth * 0.5) {
			return false;
		}

		return checkDistance(objectPosition, distance);
	}

	@Override
	protected void drawDriveArea(DrawingMap drawings)
	{
		Vector2D globalLeft = carPose.applyTo(new Vector2D(0, -carWidth * 0.5));
		Vector2D globalLeftEnd = carPose.applyTo(new Vector2D(distance + carLength, -carWidth * 0.5));
		drawings.draw("ObstacleInner", Color.YELLOW, new SubLine(globalLeft, globalLeftEnd, 0.0001));
		globalLeft = carPose.applyTo(new Vector2D(0, carWidth * 0.5));
		globalLeftEnd = carPose.applyTo(new Vector2D(distance + carLength, carWidth * 0.5));
		drawings.draw("ObstacleOuter", Color.YELLOW, new SubLine(globalLeft, globalLeftEnd, 0.0001));
	}
}
