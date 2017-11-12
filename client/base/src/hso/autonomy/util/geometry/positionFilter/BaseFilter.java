/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry.positionFilter;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Base class for position filters that provides some convenience functions
 * @author kdorer
 */
public abstract class BaseFilter implements IPositionFilter
{
	@Override
	public abstract Vector3D filterPosition(Vector3D newPosition, Vector3D oldPosition, Vector3D speed);

	@Override
	public Vector3D filterPosition(Vector3D newPosition, Vector3D oldPosition)
	{
		return filterPosition(newPosition, oldPosition, Vector3D.ZERO);
	}

	@Override
	public Vector3D filterPosition(Vector3D newPosition)
	{
		return filterPosition(newPosition, newPosition, Vector3D.ZERO);
	}

	@Override
	public void reset()
	{
	}
}
