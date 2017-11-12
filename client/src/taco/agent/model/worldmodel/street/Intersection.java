package taco.agent.model.worldmodel.street;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import taco.agent.model.worldmodel.street.SegmentUtils.LineType;

public class Intersection implements Comparable<Intersection>
{
	Vector2D point;

	SegmentUtils.LineType type;

	private double distance;

	public Intersection(Vector2D intersection, LineType type, double distance)
	{
		point = intersection;
		this.type = type;
		this.distance = distance;
	}

	public Vector2D getPoint()
	{
		return point;
	}

	public SegmentUtils.LineType getType()
	{
		return type;
	}

	public double distance(Vector2D other)
	{
		return point.distance(other);
	}

	@Override
	public int compareTo(Intersection other)
	{
		if (distance < other.distance) {
			return -1;
		} else if (distance > other.distance) {
			return 1;
		}
		return 0;
	}
}
