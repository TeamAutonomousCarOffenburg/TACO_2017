/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;

/**
 * Simple immutable class, combining a local position and a known position. This
 * implementation of a reference point is always visible.
 *
 * @author Stefan Glaser
 *
 */
public class ReferencePoint implements IReferencePoint
{
	private final Vector3D localPosition;

	private final Vector3D knownPosition;

	public ReferencePoint(Vector3D localPosition, Vector3D knownPosition)
	{
		this.localPosition = localPosition;
		this.knownPosition = knownPosition;
	}

	@Override
	public Vector3D getLocalPosition()
	{
		return localPosition;
	}

	@Override
	public Vector3D getKnownPosition()
	{
		return knownPosition;
	}

	@Override
	public boolean isVisible()
	{
		return true;
	}

	@Override
	public int compareTo(IReferencePoint o)
	{
		return 0;
	}
}
