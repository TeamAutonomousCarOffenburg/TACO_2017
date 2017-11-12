package taco.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import taco.agent.communication.perception.PerceptorName;

public class LaneMiddlePerceptor extends AudiCupPerceptor
{
	private boolean valid;

	/** how confident [0..1] we are */
	private float confidence;

	/** count since how many images we did not detect lane middle */
	private int invalidSince;

	/** the desired x pixel coordinate we want the lane middle to be */
	private int wantedX;

	/** the calculated x pixel coordinate of the lane middle */
	private int middleX;

	/** the y pixel coordinate of the line ahead */
	private int aheadLineY;

	/** the calculated lane middle in camera coordinate system */
	private Vector3D middleXCamera;

	/** the x pixel coordinate of the right line at the lane following scan line */
	private int rightLineX;

	/** the x pixel coordinate of the middle or left line at the lane following scan line */
	private int middleLineX;

	/** the x pixel coordinate of the middle or left line at the lane following scan line */
	private int leftLineX;

	/** the x pixel coordinate for starting to scan for right line */
	private int scanRightStartX;

	/** the y pixel coordinate for starting to scan for right line */
	private int scanRightStartY;

	/** the x pixel coordinate for ending to scan for right line */
	private int scanRightEndX;

	/** we believe we are in a crossing */
	private boolean inCrossing;

	public LaneMiddlePerceptor(long timestamp, int middleX, int wantedX, boolean valid, float confidence,
			int rightLineX, int middleLineX, int leftLineX)
	{
		super(PerceptorName.LANE_MIDDLE, timestamp);
		this.middleX = middleX;
		this.wantedX = wantedX;
		this.valid = valid;
		this.confidence = confidence;
		this.rightLineX = rightLineX;
		this.middleLineX = middleLineX;
		this.leftLineX = leftLineX;
	}

	public boolean isValid()
	{
		return valid;
	}

	public float getConfidence()
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

	public Vector3D getMiddleXCamera()
	{
		return middleXCamera;
	}

	public int getRightLineX()
	{
		return rightLineX;
	}

	public int getMiddleLineX()
	{
		return middleLineX;
	}

	public int getLeftLineX()
	{
		return leftLineX;
	}

	public int getScanRightStartX()
	{
		return scanRightStartX;
	}

	public int getScanRightStartY()
	{
		return scanRightStartY;
	}

	public int getScanRightEndX()
	{
		return scanRightEndX;
	}

	public boolean isInCrossing()
	{
		return inCrossing;
	}
}
