/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry.orientationFilter;

import java.util.LinkedList;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

public abstract class OrientationFilterBase implements IOrientationFilter
{
	protected static final int DEFAULT_BUFFER_SIZE = 5;

	protected final LinkedList<Rotation> filterbuffer = new LinkedList<>();

	protected int bufferSize = DEFAULT_BUFFER_SIZE;

	public OrientationFilterBase()
	{
	}

	public OrientationFilterBase(int i)
	{
		bufferSize = i;
	}

	@Override
	public Rotation filterOrientation(Rotation newOrientation)
	{
		if (newOrientation == null)
			return null;

		filterbuffer.push(newOrientation);
		if (filterbuffer.size() > bufferSize)
			filterbuffer.pollLast();

		return calculateNewOrientation();
	}

	protected abstract Rotation calculateNewOrientation();

	@Override
	public void reset()
	{
		filterbuffer.clear();
	}
}
