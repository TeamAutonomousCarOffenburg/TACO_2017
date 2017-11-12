/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.util.geometry.IPose3D;

public interface IAgentMetaModel extends Serializable {
	/**
	 * Retrieve the name of this meta model.
	 *
	 * @return the name of this model
	 */
	String getName();

	/**
	 * Creates all Effectors based on this meta-model
	 * @return a map with all effectors
	 */
	Map<String, IEffector> createEffectors();

	/**
	 * Returns a list of body part configurations, specifying the different body
	 * parts of the agent and their related sensors/effectors.
	 *
	 * @return the list of body part configurations
	 */
	List<IBodyPartConfiguration> getBodyPartConfigurations();

	/**
	 * Returns the name of the body part containing the camera. This method is
	 * used to dynamically relate the vision to the system of the root-body.
	 *
	 * @return the name of the body part containing the camera
	 */
	String getNameOfCameraBodyPart();

	/**
	 * Returns the offset of the camera relative to the system of the body part
	 * containing the camera.
	 *
	 * @return the offset of the camera in the camera body part
	 */
	IPose3D getCameraOffset();

	/**
	 * Returns an unmodifiable collection of available joints of the robot
	 * described by this meta model.
	 *
	 * @return an unmodifiable collection of available joints of the robot model
	 */
	Collection<ISensorConfiguration> getAvailableJoints();

	/**
	 * Returns a list of child body part configurations to the given body part
	 * configuration. This method is useful when browsing the robot model.
	 *
	 * @param bodyPart the parent body part configuration
	 * @return a list of child body part configurations
	 */
	List<IBodyPartConfiguration> getChildBodyConfigurations(IBodyPartConfiguration bodyPart);

	/**
	 * Returns the root body configuration. The root body configuration is the
	 * only body part configuration which has no parent body part.
	 *
	 * @return the root body configuration
	 */
	IBodyPartConfiguration getRootBodyConfiguration();

	/**
	 * Returns a string array containing the names of all available joints of the
	 * robot described by this meta model.
	 *
	 * @return an array of names to available joints of the robot model
	 */
	String[] getAvailableJointNames();

	/**
	 * @return the names of available effectors
	 */
	String[] getAvailableEffectorNames();

	float getHeight();
}
