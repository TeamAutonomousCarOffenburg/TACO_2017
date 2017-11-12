/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;

/**
 * A simple class, holding an unlabeled reference line, which can be used for
 * localization.
 *
 * @author Stefan Glaser
 *
 */
public class LocalizationLine implements ILocalizationLine
{
	private Vector3D position1;

	private Vector3D position2;

	/**
	 * Creates a simple LocalizationLine with the two positions.
	 *
	 * @param position1 - the first observed position of the line
	 * @param position2 - the second observed position of the line
	 */
	public LocalizationLine(Vector3D position1, Vector3D position2)
	{
		this.position1 = position1;
		this.position2 = position2;
	}

	@Override
	public Vector3D getLocalPosition1()
	{
		return position1;
	}

	@Override
	public Vector3D getLocalPosition2()
	{
		return position2;
	}
}
