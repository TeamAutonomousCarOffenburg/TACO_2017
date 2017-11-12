/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IVisibleObjectPerceptor;

/**
 * Visible Object
 *
 * @author Simon Raffeiner
 */
public class VisibleObjectPerceptor extends Perceptor implements IVisibleObjectPerceptor
{
	/** Object position in camera coordinate system */
	private Vector3D position;

	/** true if this perceptor has depth information in the position */
	private boolean hasDepth;

	/** how confident we are in the perception [0..1] */
	private double confidence;

	public VisibleObjectPerceptor(String name, Vector3D position, boolean hasDepth, double confidence)
	{
		super(name);
		this.position = position;
		this.hasDepth = hasDepth;
	}

	@Override
	public Vector3D getPosition()
	{
		return position;
	}

	@Override
	public void setPosition(Vector3D position)
	{
		this.position = position;
	}

	@Override
	public double getDistance()
	{
		return position.getNorm();
	}

	@Override
	public double getHorizontalAngle()
	{
		return position.getAlpha();
	}

	@Override
	public double getHorizontalAngleDeg()
	{
		return Math.toDegrees(position.getAlpha());
	}

	@Override
	public double getLatitudeAngle()
	{
		return position.getDelta();
	}

	@Override
	public double getLatitudeAngleDeg()
	{
		return Math.toDegrees(position.getDelta());
	}

	@Override
	public boolean hasDepth()
	{
		return hasDepth;
	}

	public double getConfidence()
	{
		return confidence;
	}
}
