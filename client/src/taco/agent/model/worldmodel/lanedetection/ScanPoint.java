package taco.agent.model.worldmodel.lanedetection;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;

public class ScanPoint
{
	/** x pixel coordinate */
	private int imageX;

	/** position of the line in car coordinate system (on scan line) */
	private Vector2D positionCarCoordinates;

	/** position of the line in world coordinate system */
	private Vector2D positionWorldCoordinates;

	/** position of the line on the map in car coordinates */
	private Vector2D mapPositionCar;

	/** position of the line on the map in world coordinates */
	private Vector2D mapPositionWorld;

	/**
	 * Creates an invalid scan point
	 */
	public ScanPoint()
	{
		imageX = -1;
	}

	public ScanPoint(IPose2D carPose, int imageX, Vector2D positionCarCoordinates, Vector2D mapPositionCar)
	{
		this.imageX = imageX;
		this.positionCarCoordinates = positionCarCoordinates;
		if (positionCarCoordinates != null) {
			positionWorldCoordinates = carPose.applyTo(positionCarCoordinates);
		}
		this.mapPositionCar = mapPositionCar;
		if (mapPositionCar != null) {
			mapPositionWorld = carPose.applyTo(mapPositionCar);
		} else {
			mapPositionWorld = null;
		}
	}

	/**
	 * Copies image and car coordinates from point1 and map coordinates from point2
	 * @param point1 the point with the proper pixel and car coordinates
	 * @param point2 the point with the proper map coordinates
	 */
	public ScanPoint(ScanPoint point1, ScanPoint point2)
	{
		super();
		this.imageX = point1.imageX;
		this.positionCarCoordinates = point1.positionCarCoordinates;
		this.positionWorldCoordinates = point1.positionWorldCoordinates;
		this.mapPositionCar = point2.mapPositionCar;
		this.mapPositionWorld = point2.mapPositionWorld;
	}

	public boolean isValid()
	{
		return imageX >= 0;
	}

	public void setInvalid()
	{
		imageX = -1;
	}

	public int getImageX()
	{
		return imageX;
	}

	public Vector2D getPositionCarCoordinates()
	{
		return positionCarCoordinates;
	}

	public Vector2D getPositionWorld()
	{
		return positionWorldCoordinates;
	}

	public Vector2D getMapPositionCar()
	{
		return mapPositionCar;
	}

	public Vector2D getMapPositionWorld()
	{
		return mapPositionWorld;
	}

	/**
	 * @return difference of map position to seen position in y direction of car coordinate system, Double.MAX_VALUE if
	 * one of the two is invalid
	 */
	public double getMapDeltaY()
	{
		if (!isValid() || positionCarCoordinates == null || mapPositionCar == null) {
			return Double.MAX_VALUE;
		}
		return mapPositionCar.getY() - positionCarCoordinates.getY();
	}
}
