/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

import hso.autonomy.util.geometry.Angle;

/**
 * @author David Zimmermann
 */
public interface ICompassPerceptor extends IPerceptor {
	Angle getAngle();
}
