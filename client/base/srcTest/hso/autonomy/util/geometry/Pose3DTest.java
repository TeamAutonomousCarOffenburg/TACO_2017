/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import static org.junit.Assert.assertEquals;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

import hso.autonomy.util.geometry.Pose3D;

public class Pose3DTest
{
	/**
	 * Test for {@link Pose3D#applyTo(IPose3D)}, {@link Pose3D#applyTo(Vector3D)}
	 * , {@link Pose3D#applyInverseTo(IPose3D)} and
	 * {@link Pose3D#applyInverseTo(Vector3D)}
	 */
	@Test
	public void testApplyTo()
	{
		Pose3D testee = new Pose3D(new Vector3D(3, 1, 0), new Rotation(Vector3D.PLUS_K, Math.toRadians(90)));

		// Test applyTo other pose
		Pose3D otherPose = new Pose3D(new Vector3D(1, 0, 0), new Rotation(Vector3D.PLUS_K, Math.toRadians(20)));
		Pose3D resPose = testee.applyTo(otherPose);

		assertEquals(3, resPose.getX(), 0.0001);
		assertEquals(2, resPose.getY(), 0.0001);
		assertEquals(0, Rotation.distance(new Rotation(Vector3D.PLUS_K, Math.toRadians(110)), resPose.getOrientation()),
				0.0001);

		// Test applyInverseTo other pose
		Pose3D resPoseInverse = testee.applyInverseTo(resPose);

		assertEquals(otherPose.getX(), resPoseInverse.getX(), 0.0001);
		assertEquals(otherPose.getY(), resPoseInverse.getY(), 0.0001);
		assertEquals(0, Rotation.distance(otherPose.getOrientation(), resPoseInverse.getOrientation()), 0.0001);

		// Test applyTo other position
		Vector3D otherPosition = new Vector3D(2, 1, 0);
		Vector3D resPos = testee.applyTo(otherPosition);

		assertEquals(2, resPos.getX(), 0.0001);
		assertEquals(3, resPos.getY(), 0.0001);

		// Test applyInverseTo other position
		Vector3D resPosInverse = testee.applyInverseTo(resPos);

		assertEquals(otherPosition.getX(), resPosInverse.getX(), 0.0001);
		assertEquals(otherPosition.getY(), resPosInverse.getY(), 0.0001);
	}

	/**
	 * Test for {@link Pose3D#applyInverseTo(IPose3D)} and
	 * {@link Pose3D#applyInverseTo(Vector3D)}
	 */
	@Test
	public void testApplyInverseTo()
	{
		Pose3D testee = new Pose3D(new Vector3D(3, 1, 0), new Rotation(Vector3D.PLUS_K, Math.toRadians(90)));

		// Test applyTo other pose
		Pose3D otherPose = new Pose3D(new Vector3D(1, 0, 0), new Rotation(Vector3D.PLUS_K, Math.toRadians(20)));
		Pose3D resPose = testee.applyInverseTo(otherPose);

		assertEquals(-1, resPose.getX(), 0.0001);
		assertEquals(2, resPose.getY(), 0.0001);
		assertEquals(0, Rotation.distance(new Rotation(Vector3D.PLUS_K, Math.toRadians(-70)), resPose.getOrientation()),
				0.0001);

		// Test applyTo other position
		Vector3D otherPosition = new Vector3D(2, 1, 0);
		Vector3D resPos = testee.applyInverseTo(otherPosition);

		assertEquals(0, resPos.getX(), 0.0001);
		assertEquals(1, resPos.getY(), 0.0001);
	}
}
