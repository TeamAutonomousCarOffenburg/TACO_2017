/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry.positionFilter;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Interpolate a more reliable position value for an object from a list of past,
 * less reliable values
 *
 * @author sturmflut
 */
public class PositionFilter extends PositionFilterBase
{
	public PositionFilter()
	{
		super();
	}

	/**
	 * Constructor
	 *
	 * @param i Internal buffer size for past position values
	 */
	public PositionFilter(int i)
	{
		super(i);
	}

	@Override
	protected Vector3D calculateNewPosition()
	{
		Vector3D sum = new Vector3D(0, 0, 0);
		for (Vector3D v : filterBuffer) {
			sum = sum.add(v);
		}

		return sum.scalarMultiply(1f / filterBuffer.size());
	}
}
