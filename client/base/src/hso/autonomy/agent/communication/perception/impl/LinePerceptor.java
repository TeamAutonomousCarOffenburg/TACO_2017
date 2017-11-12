/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.ILinePerceptor;

/**
 * Perceptor for line sequence
 * @author dorer
 */
public class LinePerceptor extends VisibleObjectPerceptor implements ILinePerceptor
{
	/** End point of the line (polar,perceived) */
	private final Vector3D position2;

	public LinePerceptor(String name, Vector3D pol1, Vector3D pol2, boolean hasDepth)
	{
		super(name, pol1, hasDepth, 1.0);
		this.position2 = pol2;
	}

	/**
	 * @return the position2
	 */
	@Override
	public Vector3D getPosition2()
	{
		return position2;
	}

	@Override
	public double getDistance2()
	{
		return position2.getNorm();
	}

	@Override
	public double getHorizontalAngle2()
	{
		return position2.getAlpha();
	}

	@Override
	public double getHorizontalAngleDeg2()
	{
		return Math.toDegrees(position2.getAlpha());
	}

	@Override
	public double getLatitudeAngle2()
	{
		return position2.getDelta();
	}

	@Override
	public double getLatitudeAngleDeg2()
	{
		return Math.toDegrees(position2.getDelta());
	}
}
