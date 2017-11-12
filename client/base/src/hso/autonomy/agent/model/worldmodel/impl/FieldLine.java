/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.IFieldLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;

/**
 * This class represents a simple field line with a start and end point.
 * @author Stefan Glaser
 *
 */
public class FieldLine extends VisibleObject implements IFieldLine, IReferenceLine
{
	private Vector3D localPosition1;

	private Vector3D localPosition2;

	private Vector3D position1;

	private Vector3D position2;

	private final Vector3D knownPosition1;

	private final Vector3D knownPosition2;

	private Vector3D adjustedKnownPosition1;

	private Vector3D adjustedKnownPosition2;

	public FieldLine(String name, Vector3D knownPosition1, Vector3D knownPosition2)
	{
		super(name);

		this.localPosition1 = Vector3D.ZERO;
		this.localPosition2 = Vector3D.ZERO;
		this.position1 = Vector3D.ZERO;
		this.position2 = Vector3D.ZERO;
		this.knownPosition1 = knownPosition1;
		this.knownPosition2 = knownPosition2;
		this.adjustedKnownPosition1 = knownPosition1;
		this.adjustedKnownPosition2 = knownPosition2;
	}

	/**
	 * @param pos1 first global position of the line
	 * @param pos2 second global position of the line
	 */
	public void updatePositions(Vector3D pos1, Vector3D pos2, float time)
	{
		Vector3D newPos;
		position1 = pos1;
		position2 = pos2;
		if (pos1 != null && pos2 != null) {
			newPos = pos1.add(pos2).scalarMultiply(.5);
		} else if (pos1 != null) {
			newPos = pos1;
		} else {
			newPos = pos2;
		}

		super.updateFromVision(localPosition, newPos, time);
	}

	/**
	 * @param pos1 first seen local position of the line
	 * @param pos2 second seen local position of the line
	 */
	@Override
	public void updateLocalPositions(Vector3D pos1, Vector3D pos2)
	{
		localPosition1 = pos1;
		localPosition2 = pos2;
		if (pos1 != null && pos2 != null) {
			localPosition = pos1.add(pos2).scalarMultiply(.5);
		} else if (pos1 != null) {
			localPosition = pos1;
		} else {
			localPosition = pos2;
		}
		setVisible(true);
	}

	@Override
	public void adjustKnownPosition1(Vector3D adjustedKnownPosition1)
	{
		this.adjustedKnownPosition1 = adjustedKnownPosition1;
	}

	@Override
	public void adjustKnownPosition2(Vector3D adjustedKnownPosition2)
	{
		this.adjustedKnownPosition2 = adjustedKnownPosition2;
	}

	@Override
	public Vector3D getPosition1()
	{
		return position1;
	}

	@Override
	public Vector3D getPosition2()
	{
		return position2;
	}

	@Override
	public Vector3D getLocalPosition1()
	{
		return localPosition1;
	}

	@Override
	public Vector3D getLocalPosition2()
	{
		return localPosition2;
	}

	@Override
	public Vector3D getKnownPosition1()
	{
		return knownPosition1;
	}

	@Override
	public Vector3D getKnownPosition2()
	{
		return knownPosition2;
	}

	@Override
	public Vector3D getAdjustedKnownPosition1()
	{
		return adjustedKnownPosition1;
	}

	@Override
	public Vector3D getAdjustedKnownPosition2()
	{
		return adjustedKnownPosition2;
	}
}
