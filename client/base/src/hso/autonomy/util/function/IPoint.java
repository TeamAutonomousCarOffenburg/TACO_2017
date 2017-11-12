/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.function;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * A Simple Point interface.
 *
 * @author Stefan Glaser
 */
public interface IPoint {
	/**
	 * @return the x coordinate
	 */
	float getX();

	/**
	 * @return the y coordinate
	 */
	float getY();

	/**
	 * @return get this point as Vector2D object
	 */
	Vector2D asVector2D();
}
