/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry.orientationFilter;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import hso.autonomy.util.geometry.Geometry;

public class LinearWeightedOrientationFilter extends OrientationFilterBase
{
	public LinearWeightedOrientationFilter()
	{
	}

	public LinearWeightedOrientationFilter(int i)
	{
		super(i);
	}

	@Override
	protected Rotation calculateNewOrientation()
	{
		ArrayList<Rotation> rotations = new ArrayList<>();
		int weight = filterbuffer.size();
		for (Rotation r : filterbuffer) {
			for (int i = 0; i < weight; i++) {
				rotations.add(r);
			}
			weight--;
		}

		return Geometry.getAverageRotation(rotations);
	}
}
