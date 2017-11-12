/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

/**
 * Allows to remote control an agent.
 *
 * @author Klaus Dorer
 */
public interface ICommandPerceptor extends IPerceptor {
	/**
	 * Get Command
	 *
	 * @return content of the command received
	 */
	String getCommand();
}