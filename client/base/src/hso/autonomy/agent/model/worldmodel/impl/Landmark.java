/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.ILandmark;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Non movable orientation point on the field
 *
 * @author Klaus Dorer
 */
public class Landmark extends VisibleObject implements ILandmark, IReferencePoint
{
	/** the known position of the landmark (global coordinates) */
	private final Vector3D knownPosition;

	/**
	 * Constructor
	 *
	 * @param name Landmark name
	 * @param knownPosition Landmark position
	 */
	public Landmark(String name, Vector3D knownPosition)
	{
		super(name);
		this.knownPosition = knownPosition;
	}

	@Override
	public Vector3D getKnownPosition()
	{
		return knownPosition;
	}

	/**
	 * @param localPosition the position as observed in the root body system
	 */
	public void updateLocalPosition(Vector3D localPosition)
	{
		this.localPosition = localPosition;
		setVisible(true);
	}

	@Override
	public Vector3D getPosition()
	{
		// TODO: a delegate to getKnownPosition is not necessarily good here (was
		// introduced because of IFOs). We avoid obstacles, even if we see them
		// somewhere else - in case of coming from the outside to a goal when the
		// ball lies somewhere very close to a goal post, this could lead to
		// unnecessary obstacle avoidance, even if the ball is between us and the
		// goal post. maybe:
		// if(isVisible()){ return position; } else {
		return getKnownPosition();
		// }
	}

	public Vector3D getGlobalPosition()
	{
		return super.getPosition();
	}

	@Override
	public int compareTo(IReferencePoint other)
	{
		if (localPosition.getAlpha() < other.getLocalPosition().getAlpha()) {
			return 1;
		} else if (localPosition.getAlpha() > other.getLocalPosition().getAlpha()) {
			return -1;
		}
		return 0;
	}

	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof Landmark)) {
			return false;
		}
		Landmark other = (Landmark) o;
		if (!super.equals(other)) {
			return false;
		}
		return FuzzyCompare.eq(knownPosition, other.knownPosition, 0.00001f);
	}
}