package taco.agent.model.agentmeta.impl;

import com.google.gson.annotations.SerializedName;
import hso.autonomy.util.geometry.Pose3D;

public class CameraConfiguration extends MountedSensorConfiguration
{
	@SerializedName("width")
	private int imageWidth;

	@SerializedName("height")
	private int imageHeight;

	private float focalPointX;

	private float focalPointY;

	private float focalLengthX;

	private float focalLengthY;

	public CameraConfiguration(String name, String perceptorName, Pose3D pose, int imageWidth, int imageHeight,
			float focalPointX, float focalPointY, float focalLengthX, float focalLengthY)
	{
		super(name, perceptorName, pose);
		this.imageWidth = imageWidth;
		this.imageHeight = imageHeight;
		this.focalPointX = focalPointX;
		this.focalPointY = focalPointY;
		this.focalLengthX = focalLengthX;
		this.focalLengthY = focalLengthY;
	}

	public int getImageWidth()
	{
		return imageWidth;
	}

	public int getImageHeight()
	{
		return imageHeight;
	}

	public float getFocalPointX()
	{
		return focalPointX;
	}

	public float getFocalPointY()
	{
		return focalPointY;
	}

	public float getFocalLengthX()
	{
		return focalLengthX;
	}

	public float getFocalLengthY()
	{
		return focalLengthY;
	}
}
