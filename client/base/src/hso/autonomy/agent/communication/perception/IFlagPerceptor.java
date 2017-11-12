/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

/**
 * The IFlagPerceptor represents flags of the agent.
 *
 * @author Stefan Glaser
 */
public interface IFlagPerceptor extends IPerceptor {
	/**
	 * Retrieve the flag information.
	 *
	 * @return The perceived value of the flag.
	 */
	int getValue();
}
