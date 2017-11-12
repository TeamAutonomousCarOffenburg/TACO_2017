/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.channel.impl;

import hso.autonomy.agent.communication.channel.IChannel;
import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.IChannelState;

/**
 *
 * @author kdorer
 */
public abstract class ChannelBase implements IChannel
{
	/** the channel manager to inform about new messages */
	private final IChannelManager manager;

	/** state of this connection */
	protected IChannelState state;

	public ChannelBase(IChannelManager manager)
	{
		this.manager = manager;
		state = new ChannelState();
	}

	public IChannelManager getManager()
	{
		return manager;
	}

	@Override
	public IChannelState getConnectionState()
	{
		return state;
	}

	@Override
	public boolean isConnected()
	{
		return state.isConnected();
	}
}
