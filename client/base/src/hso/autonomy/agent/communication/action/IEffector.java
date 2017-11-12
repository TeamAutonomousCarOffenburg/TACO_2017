/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action;

/**
 *
 * @author kdorer
 */
public interface IEffector {
	/**
	 * @return the name of this effector
	 */
	String getName();

	/**
	 * Sets the values of this effector as required by the specific effector
	 * @param values the number of float values required by the specific effector
	 */
	void setEffectorValues(float maxGain, float... values);

	/**
	 * Called to allow an effector to reset its state after it has been read for
	 * action.
	 */
	void resetAfterAction();
}
