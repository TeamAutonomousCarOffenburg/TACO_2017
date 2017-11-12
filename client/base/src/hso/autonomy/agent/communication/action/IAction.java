/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action;

/**
 * Interface to all actions an agent can perform on the server
 */
public interface IAction {
	/**
	 * Sets the values of the passed effector as required by the specific
	 * effector
	 * @param name name of the effector as specified in
	 * @param values the number of float values required by the specific effector
	 */
	void setEffectorValues(String name, float... values);

	/**
	 * Sends a motor command to the server. Includes commands to all universals
	 * and hinge joints. The speed of each joint has to be set before by
	 * setHingeEffectorSpeed()
	 *
	 */
	void sendAction();

	/**
	 * @param motorGain the maximal gain (stiffness) motors can have
	 */
	void setMaxGain(float motorGain);

	/**
	 * Retrieve the motorGain the maximal gain (stiffness) motors can have
	 */
	float getMaxGain();

	/**
	 * Send a single empty sync message
	 */
	void sendSync();
}