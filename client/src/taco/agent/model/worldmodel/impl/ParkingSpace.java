package taco.agent.model.worldmodel.impl;

import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Polygon;
import hso.autonomy.util.geometry.Pose2D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import taco.agent.model.worldmodel.ParkingSpaceState;

public class ParkingSpace
{
	public static final float MIN_WIDTH = 0.45f;

	public static final float MIN_DEPTH = 0.85f;

	public static final float LANE_MIDDLE_DISTANCE = 0.23f;

	private final int id;

	private final IPose2D pose;

	private ParkingSpaceState state;

	private float lastModified;

	private transient Area2D.Float area;

	public ParkingSpace(int id, IPose2D pose, ParkingSpaceState state)
	{
		this.id = id;
		this.state = state;
		this.pose = pose;
		lastModified = 0;
	}

	void update(ParkingSpaceState state, float globalTime)
	{
		if (this.state != state) {
			this.state = state;
			lastModified = globalTime;
		}
	}

	public IPose2D getPose()
	{
		return pose;
	}

	public Area2D.Float getArea()
	{
		if (area == null) {
			float halfSpace = MIN_WIDTH / 2;
			IPose2D topLeft = pose.applyTo(new Pose2D(-halfSpace, 0));
			IPose2D bottomRight = pose.applyTo(new Pose2D(halfSpace, -MIN_DEPTH));

			area = new Area2D.Float(topLeft.getX(), bottomRight.getX(), topLeft.getY(), bottomRight.getY());
		}
		return area;
	}

	public Polygon getMeasurementArea()
	{
		return getMeasurementArea(0.0, 0.0);
	}

	public Polygon getMeasurementArea(double moveX)
	{
		return getMeasurementArea(moveX, 0.0);
	}

	public Polygon getMeasurementArea(double moveX, double moveY)
	{
		double minWidthHalf = MIN_WIDTH / 2;
		IPose2D bl = this.pose.applyTo(new Pose2D(-minWidthHalf - moveX, -moveY, this.pose.getAngle()));
		IPose2D br = this.pose.applyTo(new Pose2D(+minWidthHalf - moveX, -moveY, this.pose.getAngle()));
		IPose2D tl = this.pose.applyTo(
				new Pose2D(-minWidthHalf - moveX, +LANE_MIDDLE_DISTANCE * 2.5 - moveY, this.pose.getAngle()));
		IPose2D tr = this.pose.applyTo(
				new Pose2D(+minWidthHalf - moveX, +LANE_MIDDLE_DISTANCE * 2.5 - moveY, this.pose.getAngle()));

		return new Polygon(new Vector2D(bl.getX(), bl.getY()), new Vector2D(br.getX(), br.getY()),
				new Vector2D(tr.getX(), tr.getY()), new Vector2D(tl.getX(), tl.getY()));
	}

	public int getID()
	{
		return id;
	}

	public float getLastModified()
	{
		return lastModified;
	}

	public ParkingSpaceState getState()
	{
		return state;
	}

	@Override
	public String toString()
	{
		return "\nid=" + id + ", pose=" + pose + ", area=" + getArea() + ", state=" + state +
				", lastModified=" + lastModified;
	}
}
