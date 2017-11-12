package taco.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.VectorUtils;
import taco.agent.model.agentmeta.impl.CameraConfiguration;
import taco.agent.model.agentmodel.ICameraSensor;
import taco.agent.model.worldmodel.lanedetection.LaneMiddle;

public class CameraSensor extends AudiCupSensor implements ICameraSensor
{
	private float focalPointX;

	private float focalPointY;

	private float focalHorizontal;

	private float focalVertical;

	public CameraSensor(CameraConfiguration config)
	{
		super(config.getName(), config.getPose());
		focalPointX = config.getFocalPointX();
		focalPointY = config.getFocalPointY();
		focalHorizontal = config.getFocalLengthX();
		focalVertical = config.getFocalLengthY();
	}

	@Override
	public void updateFromPerception(IPerception perception)
	{
		// for now we do not get camera perceptions in client
	}

	@Override
	public Vector3D pixelToCar(Vector2D position)
	{
		if (position.getY() <= focalPointY) {
			// this is a point above the horizon
			return null;
		}

		// Calculate pixel coordinates relative to the center of the image
		double mx = position.getX() - focalPointX;
		double my = position.getY() - focalPointY;

		// Calculate direction vector from relative pixel coordinates and focal-length
		Vector3D dir = new Vector3D(1, -mx / focalHorizontal, -my / focalVertical);

		// for now we assume the camera is not rotated

		// Scale direction vector such that it touches the ground
		Vector3D cameraPosition = dir.scalarMultiply(pose.getZ() / Math.abs(dir.getZ()));

		return pose.applyTo(cameraPosition);
	}

	@Override
	public Vector2D carToPixel(Vector2D position)
	{
		return carToPixel(VectorUtils.to3D(position));
	}

	@Override
	public Vector2D carToPixel(Vector3D position)
	{
		Vector3D cameraPosition = pose.applyInverseTo(position);
		if (cameraPosition.getX() == 0) {
			return null;
		}

		double scale = 1.0 / cameraPosition.getX();

		double mx = cameraPosition.getY() * scale * focalHorizontal;
		double my = cameraPosition.getZ() * scale * focalVertical;

		return new Vector2D(-mx + focalPointX, -my + focalPointY);
	}

	@Override
	public Area2D.Float pixelToCar(Area2D.Int area)
	{
		Vector3D bottomLeft = pixelToCar(area.getBottomLeft());
		Vector3D bottomRight = pixelToCar(area.getBottomRight());
		if (bottomLeft == null || bottomRight == null) {
			return null;
		}

		double height = LaneMiddle.pixelToCamera(area.getHeight());
		return new Area2D.Float(bottomLeft.getX() - height, bottomRight.getX(), bottomLeft.getY(), bottomRight.getY());
	}
}
