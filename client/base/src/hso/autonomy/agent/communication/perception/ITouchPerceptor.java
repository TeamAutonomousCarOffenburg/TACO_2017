/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

/**
 * The Touch Perceptor represents a "digital" input: It either touches something
 * (probably the ground) or it doesn't.
 *
 * @author Simon Raffeiner
 */
public interface ITouchPerceptor extends IPerceptor {
	/**
	 * Get perceptor state
	 *
	 * @return True if touched, False if not
	 */
	boolean getState();
}