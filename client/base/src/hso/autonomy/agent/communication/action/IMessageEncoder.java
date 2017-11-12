/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action;

import java.util.Map;

/**
 * Interface for the protocol encoding layer
 * @author kdorer
 */
public interface IMessageEncoder {
	byte[] encodeMessage(Map<String, IEffector> effectors);
}
