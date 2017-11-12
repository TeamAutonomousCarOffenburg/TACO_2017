/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action;

import java.util.Map;

/**
 *
 * @author kdorer
 */
public interface IActionPerformer {
	void performAction(Map<String, IEffector> effectors);
}
