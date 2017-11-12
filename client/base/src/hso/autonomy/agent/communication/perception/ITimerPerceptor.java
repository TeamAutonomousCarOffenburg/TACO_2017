/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

/**
 * The Timer Perceptor represents the time of the virtual global clock.
 *
 * @author Simon Raffeiner
 */
public interface ITimerPerceptor extends IPerceptor {
	float getTime();
}