/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.channel;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;

/**
 * @author kdorer
 */
public interface IOutputChannel extends IChannel {
	void sendMessage(Map<String, IEffector> effectors);
}
