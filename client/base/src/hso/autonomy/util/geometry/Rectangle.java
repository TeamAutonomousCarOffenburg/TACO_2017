/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import java.io.Serializable;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * A rectangular Area in 2D space. This is a mutable version of Area2D.Int
 *
 * @author Klaus Dorer
 */
public class Rectangle implements Serializable
{
	private int minX;

	private int maxX;

	private int minY;

	private int maxY;

	public Rectangle(int x, int y)
	{
		this(x, x, y, y);
	}

	public Rectangle(int minX, int maxX, int minY, int maxY)
	{
		if (minX > maxX) {
			int tmp = maxX;
			maxX = minX;
			minX = tmp;
		}

		if (minY > maxY) {
			int tmp = maxY;
			maxY = minY;
			minY = tmp;
		}

		this.minX = minX;
		this.maxX = maxX;
		this.minY = minY;
		this.maxY = maxY;
	}

	public int getMinX()
	{
		return minX;
	}

	public int getMaxX()
	{
		return maxX;
	}

	public int getMinY()
	{
		return minY;
	}

	public int getMaxY()
	{
		return maxY;
	}

	public int getWidth()
	{
		return maxX - minX;
	}

	public int getHeight()
	{
		return maxY - minY;
	}

	public Vector2D getTopLeft()
	{
		return new Vector2D(minX, minY);
	}

	public Vector2D getTopRight()
	{
		return new Vector2D(maxX, minY);
	}

	public Vector2D getBottomLeft()
	{
		return new Vector2D(minX, maxY);
	}

	public Vector2D getBottomRight()
	{
		return new Vector2D(maxX, maxY);
	}

	public boolean contains(int x, int y)
	{
		return x >= minX && x <= maxX && y >= minY && y <= maxY;
	}

	public boolean containsX(int x)
	{
		return x >= minX && x <= maxX;
	}

	public boolean containsY(int y)
	{
		return y >= minY && y <= maxY;
	}

	public boolean isInside(Rectangle other)
	{
		return other.contains(minX, minY) && other.contains(maxX, maxY);
	}

	@Override
	public boolean equals(Object obj)
	{
		if (super.equals(obj)) {
			return true;
		}

		if (obj instanceof Rectangle) {
			Rectangle other = (Rectangle) obj;
			return other.minX == minX && other.maxX == maxX && other.minY == minY && other.maxY == maxY;
		}

		return false;
	}

	@Override
	public String toString()
	{
		return "[minX=" + minX + ",maxX=" + maxX + ",minY=" + minY + ",maxY=" + maxY + "]";
	}

	public void setMinX(int x)
	{
		minX = x;
	}

	public void setMaxX(int x)
	{
		maxX = x;
	}

	public void setMinY(int y)
	{
		minY = y;
	}

	public void setMaxY(int y)
	{
		maxY = y;
	}
}
