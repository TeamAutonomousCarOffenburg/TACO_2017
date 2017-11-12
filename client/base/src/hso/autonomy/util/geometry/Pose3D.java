/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import java.text.DecimalFormat;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Stores a 3D position and orientation
 *
 * @author Klaus Dorer
 */
public class Pose3D implements IPose3D
{
	/** The 3D position. */
	public Vector3D position;

	/** The 3D orientation. */
	public Rotation orientation;

	/**
	 * Instantiate a new Pose3D object. The position is initialized with the
	 * {@link Vector3D#ZERO} vector and the {@link Rotation#IDENTITY} rotation.
	 */
	public Pose3D()
	{
		this(Vector3D.ZERO, Rotation.IDENTITY);
	}

	/**
	 * Instantiate a new Pose3D object and initialize position and orientation.
	 *
	 * @param position: the position
	 * @param orientation: the orientation
	 */
	public Pose3D(Vector3D position, Rotation orientation)
	{
		this.position = position;
		this.orientation = orientation;
	}

	@Override
	public double getX()
	{
		return position.getX();
	}

	@Override
	public double getY()
	{
		return position.getY();
	}

	@Override
	public double getZ()
	{
		return position.getZ();
	}

	@Override
	public Vector3D getPosition()
	{
		return position;
	}

	@Override
	public Rotation getOrientation()
	{
		return orientation;
	}

	@Override
	public Pose3D applyTo(IPose3D other)
	{
		return new Pose3D(												//
				position.add(orientation.applyTo(other.getPosition())), //
				orientation.applyTo(other.getOrientation()));
	}

	@Override
	public Pose3D applyInverseTo(IPose3D other)
	{
		return new Pose3D(															//
				orientation.applyInverseTo(other.getPosition().subtract(position)), //
				orientation.applyInverseTo(other.getOrientation()));
	}

	@Override
	public Vector3D applyTo(Vector3D pos)
	{
		return position.add(orientation.applyTo(pos));
	}

	@Override
	public Vector3D applyInverseTo(Vector3D pos)
	{
		return orientation.applyInverseTo(pos.subtract(position));
	}

	@Override
	public boolean equals(Object obj)
	{
		if (obj instanceof IPose3D) {
			IPose3D other = (IPose3D) obj;
			return position.equals(other.getPosition()) &&
					Rotation.distance(orientation, other.getOrientation()) < 0.0001;
		}

		return super.equals(obj);
	}

	@Override
	public String toString()
	{
		double[][] mat = orientation.getMatrix();

		return position.toString(new DecimalFormat("#.##")) + "\n" +
				String.format(							  //
						"\t%2.3f %2.3f %2.3f\n"			  //
								+ "\t%2.3f %2.3f %2.3f\n" //
								+ "\t%2.3f %2.3f %2.3f",  //
						mat[0][0], mat[0][1], mat[0][2],  //
						mat[1][0], mat[1][1], mat[1][2],  //
						mat[2][0], mat[2][1], mat[2][2]);
	}
}
