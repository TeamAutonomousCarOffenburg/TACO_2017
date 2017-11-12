/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import hso.autonomy.agent.model.agentmodel.IJoint;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.geometry.Geometry;

/**
 * @author stilnox
 *
 */
public class BodyPartTest
{
	private IJoint joint2;

	private IJoint joint3;

	private IJoint joint4;

	private IJoint joint5;

	private BodyPart torso;

	private BodyPart shoulderR;

	private BodyPart lowerarm;

	private BodyPart hand;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
	}

	@Test
	public void test1()
	{
		joint2 = createJointMockNoRotation("joint2Name");
		joint3 = createJointMockNoRotation("joint3Name");

		torso = new BodyPart("torso", null, null, Vector3D.ZERO, Vector3D.ZERO, 0f, null);

		BodyPart part1 = new BodyPart("part1", torso, joint2, new Vector3D(10, 0, 0), Vector3D.ZERO, 0f, null);

		BodyPart part2 = new BodyPart("part2", part1, joint3, Vector3D.ZERO, new Vector3D(10, 0, 0), 0f, null);

		assertEquals(new Vector3D(10, 0, 0), part2.getPosition());
	}

	@Test
	public void test2()
	{
		joint2 = createJointMockNoRotation("joint2Name");
		joint3 = createJointMockNoRotation("joint3Name");

		torso = new BodyPart("torso", null, null, Vector3D.ZERO, Vector3D.ZERO, 0f, null);

		BodyPart part1 = new BodyPart("part1", torso, joint2, new Vector3D(10, 0, 0), Vector3D.ZERO, 0f, null);

		BodyPart part2 = new BodyPart("part2", part1, joint3, new Vector3D(0, 10, 0), Vector3D.ZERO, 0f, null);

		assertEquals(new Vector3D(10, 10, 0), part2.getPosition());
	}

	@Test
	public void test3()
	{
		createJointMocks();
		createBodyParts();

		Vector3D handPos = hand.getPosition();

		assertEquals(0.108, handPos.getX(), 0.00001);
		assertEquals(0.195, handPos.getY(), 0.00001);
		assertEquals(0.084, handPos.getZ(), 0.00001);
	}

	@Test
	public void test4()
	{
		createJointMocks();
		joint2 = createJointMock(Geometry.createXRotation(Math.toRadians(90)), "joint2Name");
		createBodyParts();

		Vector3D position = lowerarm.getPosition();

		assertEquals(0.098, position.getX(), 0.0001);
		assertEquals(-0.009, position.getY(), 0.0001);
		assertEquals(0.215, position.getZ(), 0.0001);
	}

	@Test
	public void testCopyConstructor()
	{
		createTorsoAndJoint2();
		joint3 = mock(IJoint.class);
		when(joint2.copy()).thenReturn(joint3);

		shoulderR = new BodyPart("shoulderR", torso, joint2, new Vector3D(1.0, 2.0, 3.0), Vector3D.ZERO, 0.5f, null);

		BodyPart copyTorso = new BodyPart(torso, null);
		BodyPart copyShoulder = copyTorso.getChild("shoulderR");

		assertEquals(1, copyTorso.getNoOfChildren());
		assertSame(copyTorso, copyShoulder.getParent());
		assertNotSame(torso, copyTorso);
		assertNotSame(shoulderR, copyShoulder);
		assertNotSame(shoulderR.getJoint(), copyShoulder.getJoint());

		assertNotSame(torso.getJointTransformation(), copyTorso.getJointTransformation());
		assertEquals(torso.getJointTransformation(), copyTorso.getJointTransformation());
	}

	@Test
	public void testGetBodyPart()
	{
		createJointMocks();
		createBodyParts();

		BodyPart result = torso.getBodyPart("hand");
		assertSame(hand, result);
	}

	@Test
	public void testGetCenterOfMass()
	{
		createTorsoAndJoint2();
		shoulderR = new BodyPart("shoulderR", torso, joint2, new Vector3D(1.0, 2.0, 3.0), Vector3D.ZERO, 0.5f, null);

		Vector3D result = torso.getCenterOfMass();
		assertEquals(0.2f, result.getX(), 0.00001);
		assertEquals(0.4f, result.getY(), 0.00001);
		assertEquals(0.6f, result.getZ(), 0.00001);
	}

	private void createJointMocks()
	{
		joint2 = createJointMockNoRotation("joint2Name");
		joint3 = createJointMockNoRotation("joint3Name");
		joint4 = createJointMockNoRotation("joint4Name");
		joint5 = createJointMockNoRotation("joint5Name");
	}

	private IJoint createJointMockNoRotation(String name)
	{
		return createJointMock(Rotation.IDENTITY, name);
	}

	@SuppressWarnings("unchecked")
	private IJoint createJointMock(Rotation rotation, String name)
	{
		IJoint joint = mock(IJoint.class);
		when(joint.getRotation()).thenReturn(rotation);
		when(joint.getName()).thenReturn(name);
		joint.updateSensors(Mockito.anyObject(), Mockito.anyObject());
		return joint;
	}

	private void createBodyParts()
	{
		torso = new BodyPart("torso", null, null, Vector3D.ZERO, Vector3D.ZERO, 0f, null);

		shoulderR = new BodyPart("shoulderR", torso, joint2, new Vector3D(0.098, 0, 0.075), Vector3D.ZERO, 0f, null);

		BodyPart upperarmR = new BodyPart(
				"upperarmR", shoulderR, joint3, new Vector3D(0.01, 0.02, 0), new Vector3D(-0.01, -0.02, 0), 0f, null);

		BodyPart elbow =
				new BodyPart("elbow", upperarmR, joint4, new Vector3D(-0.01, 0.07, 0.009), Vector3D.ZERO, 0f, null);

		lowerarm =
				new BodyPart("lowerarm", elbow, joint5, new Vector3D(0, 0.05, 0), new Vector3D(0, -0.05, 0), 0f, null);

		hand = new BodyPart("hand", lowerarm, null, new Vector3D(0.01, 0.055, 0), Vector3D.ZERO, 0f, null);
	}

	@SuppressWarnings("unchecked")
	private void createTorsoAndJoint2()
	{
		joint2 = mock(IJoint.class);
		when(joint2.getRotation()).thenReturn(Rotation.IDENTITY);
		when(joint2.getName()).thenReturn("joint2Name");
		joint2.updateSensors(Mockito.anyObject(), Mockito.anyObject());

		torso = new BodyPart("torso", null, null, Vector3D.ZERO, Vector3D.ZERO, 2.0f, null);
	}
}
