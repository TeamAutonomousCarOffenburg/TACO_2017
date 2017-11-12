package taco.agent.model.worldmodel.lanedetection;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.SubLine;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.VectorUtils;
import taco.agent.communication.perception.impl.LaneMiddlePerceptor;
import taco.agent.model.worldmodel.street.SegmentUtils;
import taco.agent.model.worldmodel.street.StreetMap;

public class LaneMiddle
{
	public static final double SCANLINE_DISTANCE = 0.95;

	public static final double SCANLINE_WIDTH = 2;

	public static SubLine SCANLINE = new SubLine(new Vector2D(LaneMiddle.SCANLINE_DISTANCE, -SCANLINE_WIDTH / 2.0),
			new Vector2D(LaneMiddle.SCANLINE_DISTANCE, SCANLINE_WIDTH / 2.0), 0.0001);

	public static final double METER_TO_PIXEL = 449;

	private boolean valid;

	private double confidence;

	/** count since how many images we did not detect lane middle */
	private int invalidSince;

	/** the desired x pixel coordinate we want the lane middle to be */
	private int wantedX;

	/** the calculated x pixel coordinate of the lane middle */
	private int middleX;

	/** the y pixel coordinate of the line ahead */
	private int aheadLineY;

	/** the line points identified in the camera image */
	private ScanPoint[] scanPoints;

	/** true if the client changed matching of right, middle or left */
	private boolean swapped;

	public LaneMiddle()
	{
		scanPoints = new ScanPoint[3];
		for (int i = 0; i < scanPoints.length; i++) {
			scanPoints[i] = new ScanPoint();
		}
	}

	public void update(LaneMiddlePerceptor perceptor, IPose2D carPose, StreetMap map)
	{
		valid = perceptor.isValid();
		confidence = perceptor.getConfidence();
		wantedX = perceptor.getWantedX();
		middleX = perceptor.getMiddleX();
		aheadLineY = perceptor.getAheadLineY();
		invalidSince = perceptor.getInvalidSince();

		Vector3D[] lines = SegmentUtils.getLinePositions(carPose, LaneMiddle.SCANLINE, map);
		int[] pixelX = {perceptor.getRightLineX(), perceptor.getMiddleLineX(), perceptor.getLeftLineX()};
		for (int i = 0; i < lines.length; i++) {
			Vector2D position = getPositionInCarCoordinates(pixelX[i]);
			scanPoints[i] = new ScanPoint(carPose, pixelX[i], position, VectorUtils.to2D(lines[i]));
		}

		swapped = false;
	}

	public void setValid(boolean value)
	{
		valid = value;
	}

	public boolean isValid()
	{
		return valid;
	}

	public double getConfidence()
	{
		return confidence;
	}

	public int getInvalidSince()
	{
		return invalidSince;
	}

	public int getWantedX()
	{
		return wantedX;
	}

	public int getMiddleX()
	{
		return middleX;
	}

	public int getAheadLineY()
	{
		return aheadLineY;
	}

	public int getDeltaX()
	{
		return middleX - wantedX;
	}

	public int getLeftDeltaX()
	{
		int x = getLeftMiddleX();
		if (x < 0) {
			return -1;
		}
		return x - wantedX;
	}

	public int getLeftMiddleX()
	{
		if (!isValid()) {
			return -1;
		}
		int rightLineX = scanPoints[0].getImageX();
		int middleLineX = scanPoints[1].getImageX();
		int leftLineX = scanPoints[2].getImageX();
		if (middleLineX >= 0 && leftLineX >= 0) {
			return leftLineX + (middleLineX - leftLineX) / 2;
		}

		if (rightLineX >= 0 && leftLineX >= 0) {
			return leftLineX + (rightLineX - leftLineX) / 4;
		}

		if (rightLineX >= 0 && middleLineX >= 0) {
			return middleLineX - (rightLineX - middleLineX) / 2;
		}

		return -1;
	}

	@Override
	public String toString()
	{
		return "LaneMiddle [valid=" + valid + ", confidence=" + confidence + ", invalidSince=" + invalidSince +
				", wantedX=" + wantedX + ", middleX=" + middleX + ", aheadLineY=" + aheadLineY +
				", swapped=" + swapped + "]";
	}

	public static double pixelToCamera(int x)
	{
		// the values here are manually tuned
		return x / METER_TO_PIXEL;
	}

	public static int cameraToPixel(double y)
	{
		// the values here are manually tuned
		return (int) (y * METER_TO_PIXEL);
	}

	public void swapRightMiddle()
	{
		scanPoints[2] = new ScanPoint(scanPoints[1], scanPoints[2]);
		scanPoints[1] = new ScanPoint(scanPoints[0], scanPoints[1]);
		scanPoints[0] = new ScanPoint();
		middleX = (int) (scanPoints[1].getImageX() + (scanPoints[1].getImageX() - scanPoints[2].getImageX()) * 0.5);
		aheadLineY = -1;
		swapped = true;
	}

	public void swapMiddleLeft()
	{
		scanPoints[0] = new ScanPoint(scanPoints[1], scanPoints[0]);
		scanPoints[1] = new ScanPoint(scanPoints[2], scanPoints[1]);
		scanPoints[2] = new ScanPoint();
		middleX = (int) (scanPoints[0].getImageX() + scanPoints[1].getImageX() * 0.5);
		aheadLineY = -1;
		swapped = true;
	}

	public boolean isSwapped()
	{
		return swapped;
	}

	protected Vector2D getPositionInCarCoordinates(int pixelX)
	{
		if (pixelX < 0) {
			return null;
		}
		return new Vector2D(LaneMiddle.SCANLINE_DISTANCE, LaneMiddle.pixelToCamera(wantedX - pixelX));
	}

	/**
	 * @return the viewed right line position in car coordinate system, null if not valid
	 */
	public Vector2D getRightLinePosition()
	{
		return scanPoints[0].getPositionCarCoordinates();
	}
	/**
	 * @return the viewed middle line position in car coordinate system, null if not valid
	 */
	public Vector2D getMiddleLinePosition()
	{
		return scanPoints[1].getPositionCarCoordinates();
	}
	/**
	 * @return the viewed left line position in car coordinate system, null if not valid
	 */
	public Vector2D getLeftLinePosition()
	{
		return scanPoints[2].getPositionCarCoordinates();
	}

	/**
	 * @return the calculated middle of our lane position in car coordinate system, null if not valid
	 */
	public Vector2D getMiddleLanePosition()
	{
		if (middleX < 0) {
			return null;
		}
		return getPositionInCarCoordinates(middleX);
	}

	/**
	 * @return the calculated middle of the other lane position in car coordinate system, null if not valid
	 */
	public Vector2D getLeftMiddleLanePosition()
	{
		int x = getLeftMiddleX();
		if (x < 0) {
			return null;
		}
		return getPositionInCarCoordinates(x);
	}

	protected double calculateAverageDistance()
	{
		double avgDistanceNew = 0;
		int avgNewCount = 0;
		for (ScanPoint point : scanPoints) {
			double distance = point.getMapDeltaY();
			if (distance < Double.MAX_VALUE) {
				avgDistanceNew = (avgDistanceNew * avgNewCount + Math.abs(distance)) / ++avgNewCount;
			}
		}
		return avgDistanceNew;
	}

	/**
	 * @return the distances of the seen line points compared to the positions from the map, -1 if not valid
	 */
	public double[] getDistancesToMapMiddle()
	{
		double[] result = new double[3];
		for (int i = 0; i < result.length; i++) {
			if (scanPoints[i].isValid() && scanPoints[i].getPositionCarCoordinates() != null &&
					scanPoints[1].getMapPositionCar() != null) {
				result[i] = scanPoints[1].getMapPositionCar().distance(scanPoints[i].getPositionCarCoordinates());
			} else {
				result[i] = -1;
			}
		}
		return result;
	}

	public int getMiddleLineX()
	{
		return scanPoints[1].getImageX();
	}

	public double getGlobalScanPointDistance(int point1, int point2)
	{
		if (!checkPoint(point1)) {
			return -1;
		}
		if (!checkPoint(point2)) {
			return -1;
		}
		return scanPoints[point1].getPositionWorld().distance(scanPoints[point2].getPositionWorld());
	}

	private boolean checkPoint(int point)
	{
		if (point < 0 || point > 2) {
			System.err.println("Invalid scan point index: " + point);
			return false;
		}
		if (!scanPoints[point].isValid() || scanPoints[point].getPositionWorld() == null) {
			return false;
		}
		return true;
	}
}
