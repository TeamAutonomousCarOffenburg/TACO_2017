/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.impl;

import java.io.Serializable;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.IVisibleObject;
import hso.autonomy.agent.model.worldmodel.InformationSource;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.VectorUtils;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Base class for all visible objects on the field
 */
public abstract class VisibleObject implements IVisibleObject, Serializable
{
	private final String name;

	/** the last perceived local position in the root body system */
	protected Vector3D localPosition;

	/** the current position in the global coordinate system */
	protected Vector3D position;

	/** the previous position in the global coordinate system */
	protected Vector3D previousPosition;

	/** time stamp when this object was last updated by perception */
	protected float lastSeenTime;

	/** true if this object was seen in last perception */
	private boolean visible;

	/** where we know this piece of information from */
	private InformationSource informationSource;

	/**
	 * Default constructor, initializes all fields to zero/null
	 *
	 * @param name Object name
	 */
	public VisibleObject(String name)
	{
		this.name = name;
		localPosition = Vector3D.ZERO;
		position = Vector3D.ZERO;
		previousPosition = Vector3D.ZERO;
		visible = false;
		lastSeenTime = 0;
		informationSource = InformationSource.NONE;
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public Vector3D getPosition()
	{
		return position;
	}

	/**
	 * Setting this objects position is usually done when updating the object.
	 * This method is used to set the ball position from hear message only.
	 *
	 * @param pos New position
	 */
	public void setPosition(Vector3D pos)
	{
		previousPosition = position;
		position = pos;
	}

	@Override
	public Vector3D getPreviousPosition()
	{
		return previousPosition;
	}

	@Override
	public float getLastSeenTime()
	{
		return lastSeenTime;
	}

	/**
	 * Updates this object with the latest perception
	 *
	 * @param localPosition - the local position as observed in the root body
	 *        system
	 * @param globalPosition - the calculated global position of this visible
	 *        object
	 * @param time - time the current absolute time
	 */
	public void updateFromVision(Vector3D localPosition, Vector3D globalPosition, float time)
	{
		this.localPosition = localPosition;
		setPosition(globalPosition);

		visible = true;
		lastSeenTime = time;
		informationSource = InformationSource.VISION;
	}

	/**
	 * Updates this object's position due to hearing
	 * @param globalPosition the position of the object in global coordinates
	 */
	public void updateFromAudio(Vector3D localPosition, Vector3D globalPosition, float time)
	{
		this.localPosition = localPosition;
		setPosition(globalPosition);
		lastSeenTime = time;
		informationSource = InformationSource.AUDIO;
	}

	/**
	 * @param other the point to which to calculate
	 * @return the distance projection to the x y plane between this object and
	 *         the passed coordinate
	 */
	@Override
	public double getDistanceToXY(Vector3D other)
	{
		return VectorUtils.getDistanceBetweenXY(getPosition(), other);
	}

	/**
	 * @param other the point to which to calculate
	 * @return the distance projection to the x y plane between this object and
	 *         the passed coordinate
	 */
	@Override
	public double getDistanceToXY(IVisibleObject other)
	{
		return getDistanceToXY(other.getPosition());
	}

	/**
	 * Calculates the distance of this visible object to the passed one
	 * @param other the object to which to calculate distance
	 * @return the distance of this visible object to the passed one
	 */
	@Override
	public double getDistanceToXYZ(IVisibleObject other)
	{
		return getDistanceToXYZ(other.getPosition());
	}

	/**
	 * Calculates the distance of this visible object to the passed position
	 * @param other the position to which to calculate distance
	 * @return the distance of this visible object to the passed position
	 */
	@Override
	public double getDistanceToXYZ(Vector3D other)
	{
		return getPosition().subtract(other).getNorm();
	}

	/**
	 * Calculates the Direction of this visible object to the passed one
	 * @param other the object to which to calculate direction
	 * @return the direction (rad) of this visible object to the passed one
	 */
	@Override
	public Angle getDirectionTo(IVisibleObject other)
	{
		return getDirectionTo(other.getPosition());
	}

	/**
	 * Calculates the global absolute Direction of this visible object to the
	 * passed Vector3D
	 * @param other the Vector3D to which to calculate direction
	 * @return the global absolute direction (rad) of this visible object to the
	 *         passed Vector
	 */
	@Override
	public Angle getDirectionTo(Vector3D other)
	{
		return VectorUtils.getDirectionTo(getPosition(), other);
	}

	@Override
	public Vector3D getLocalPosition()
	{
		return localPosition;
	}

	public void setVisible(boolean state)
	{
		visible = state;
	}

	@Override
	public boolean isVisible()
	{
		return visible;
	}

	@Override
	public float getAge(float currentTime)
	{
		return currentTime - lastSeenTime;
	}

	@Override
	public boolean equals(Object other)
	{
		if (!(other instanceof VisibleObject)) {
			return false;
		}
		VisibleObject otherVis = (VisibleObject) other;

		if (!FuzzyCompare.eq(position, otherVis.position, 0.00001f)) {
			return false;
		}
		if (!FuzzyCompare.eq(previousPosition, otherVis.previousPosition, 0.00001f)) {
			return false;
		}
		return FuzzyCompare.eq(lastSeenTime, otherVis.lastSeenTime, 0.00001f);
	}

	@Override
	public double getCollisionDistance()
	{
		// default value for all visible objects
		return 0.7;
	}

	/**
	 * @return the source where we got this information from
	 */
	@Override
	public InformationSource getInformationSource()
	{
		return informationSource;
	}

	@Override
	public int hashCode()
	{
		return name.hashCode() + localPosition.hashCode() + position.hashCode() + previousPosition.hashCode();
	}

	public void updateNoVision(Vector3D newPosition, float globalTime)
	{
		position = newPosition;
		visible = false;
	}
}