/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Before;
import org.junit.Test;

import hso.autonomy.agent.communication.perception.ICompositeJointPerceptor;
import hso.autonomy.agent.communication.perception.IHingeJointPerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.communication.perception.impl.CompositeJointPerceptor;
import hso.autonomy.agent.communication.perception.impl.HingeJointPerceptor;
import hso.autonomy.agent.model.agentmeta.ICompositeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IHingeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IJointToMotorMapper;
import hso.autonomy.agent.model.agentmeta.impl.CompositeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.impl.HingeJointConfiguration;
import hso.autonomy.agent.model.agentmodel.IHingeJoint;
import hso.autonomy.agent.model.agentmodel.IJoint;

/**
 *
 * @author kdorer
 */
public class CompositeJointTest
{
	private CompositeJoint testee;

	private IPerception perceptionMock;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
		perceptionMock = mock(IPerception.class);

		IJointToMotorMapper mapper = new IJointToMotorMapper() {

			@Override
			public double[] motorToJointAngle(double[] motorAngles)
			{
				double[] result = new double[motorAngles.length];
				for (int i = 0; i < motorAngles.length; i++) {
					result[i] = -0.5 * motorAngles[i];
				}
				return result;
			}

			@Override
			public double[] jointToMotorAngle(double[] jointAngles)
			{
				return motorToJointAngle(jointAngles);
			}
		};
		HingeJointConfiguration hip1 =
				new HingeJointConfiguration("joint1", "p1", "e1", Vector3D.PLUS_J, -20, 20, 7, 8, 11, false);
		HingeJointConfiguration hip2 =
				new HingeJointConfiguration("joint2", "p2", "e2", Vector3D.PLUS_J, -20, 20, 7, 8, 12, false);
		IHingeJointConfiguration[] hjConfs = {hip1, hip2};
		ICompositeJointConfiguration jointConfig = new CompositeJointConfiguration("test", mapper, hjConfs);

		testee = new CompositeJoint(jointConfig);
	}

	@Test
	public void testUpdateFromPerception()
	{
		HingeJointPerceptor h1 = new HingeJointPerceptor("h1", 10, 1, 2, 3, 4, 5, 6, (byte) 7);
		HingeJointPerceptor h2 = new HingeJointPerceptor("h2", 20, 1, 2, 3, 4, 5, 6, (byte) 7);
		IHingeJointPerceptor[] perceptors = {h1, h2};
		ICompositeJointPerceptor perceptor = new CompositeJointPerceptor("testPercept", perceptors);
		when(perceptionMock.getCompositeJointPerceptor("test")).thenReturn(perceptor);

		testee.updateFromPerception(perceptionMock);

		List<IJoint> allJoints = testee.getAllSubJoints();
		assertEquals(2, allJoints.size());
		assertEquals(-5, ((IHingeJoint) allJoints.get(0)).getAngle(), 0.0001);
		assertEquals(-10, ((IHingeJoint) allJoints.get(1)).getAngle(), 0.0001);
	}

	@Test
	public void testGenerateJointAction()
	{
		Map<String, float[]> actions = new HashMap<>();

		testee.setFutureValues(0, 4, 2, 1);
		testee.setFutureValues(1, 5, -1, -4);
		testee.generateJointAction(actions);
		assertEquals(1, actions.size());
		float[] values = actions.get("test");
		assertEquals(10, values.length);
		assertEquals(-2, values[0], 0.0001);
		assertEquals(-2, values[1], 0.0001);
		assertEquals(-1, values[2], 0.0001);
		assertEquals(-0.5, values[3], 0.0001);
		assertEquals(11, values[4], 0.0001);

		assertEquals(-2.5, values[5], 0.0001);
		assertEquals(-2.5, values[6], 0.0001);
		assertEquals(0.5, values[7], 0.0001);
		assertEquals(2, values[8], 0.0001);
		assertEquals(12, values[9], 0.0001);
	}
}
