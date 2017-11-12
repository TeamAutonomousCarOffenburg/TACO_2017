package taco.agent.model.worldmodel.impl;

import hso.autonomy.agent.model.worldmodel.impl.VisibleObject;
import hso.autonomy.util.geometry.Area2D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import taco.agent.communication.perception.RecognizedObjectType;

import static hso.autonomy.util.geometry.VectorUtils.to3D;

public class Obstacle extends VisibleObject
{
	private static final int TTL = 1;

	private float perceptionTime;

	private final RecognizedObjectType type;

	/** position of the obstacle in the global coordinate system */
	private final Area2D.Float area;

	public Obstacle(float perceptionTime, RecognizedObjectType type, Area2D.Float area)
	{
		super(type.name());
		this.perceptionTime = perceptionTime;
		this.type = type;
		this.area = area;
	}

	public RecognizedObjectType getType()
	{
		return type;
	}

	public Area2D.Float getArea()
	{
		return area;
	}

	@Override
	public Vector3D getPosition()
	{
		return to3D(area.getCenter());
	}

	public boolean isValid(float currentTime)
	{
		return currentTime - perceptionTime < TTL;
	}

	@Override
	public String toString()
	{
		return type.name();
	}
}
