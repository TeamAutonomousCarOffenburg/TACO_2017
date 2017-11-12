/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.channel;

import java.util.Map;

import hso.autonomy.agent.communication.action.IActionPerformer;
import hso.autonomy.agent.communication.perception.IPerceptor;

/**
 *
 * @author kdorer
 */
public interface IChannelManager extends IActionPerformer {
	enum ChannelManagerStatus
	{
		CREATED,
		STARTED,
		STOPPED,
		LOST_MAIN_CONNECTION
	}

	void addPerceptors(Map<String, IPerceptor> newPerceptors);

	void addInputChannel(IInputChannel channel, boolean isMainChannel);

	void addOutputChannel(IOutputChannel channel);

	boolean start();

	void stop();

	boolean isConnected();

	/**
	 * @return the next map of perceptors to process, null if there is none
	 */
	Map<String, IPerceptor> getNextPerceptorMap();

	/**
	 * Called if a connection of an input channel is lost
	 */
	void lostConnection(IInputChannel channel);

	ChannelManagerStatus getStatus();

	/**
	 * Whether there was a connection at any point
	 */
	boolean hadConnection();

	IInputChannel getMainChannel();
}
