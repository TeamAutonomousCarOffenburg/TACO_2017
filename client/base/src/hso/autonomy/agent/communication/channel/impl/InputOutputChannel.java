/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.channel.impl;

import java.util.Map;

import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.agent.communication.action.IMessageEncoder;
import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.IOutputChannel;
import hso.autonomy.agent.communication.perception.IMessageParser;
import hso.autonomy.util.connection.ConnectionException;
import hso.autonomy.util.connection.IServerConnection;

/**
 *
 * @author kdorer
 */
public abstract class InputOutputChannel extends InputChannel implements IOutputChannel
{
	/** message encoder */
	private final IMessageEncoder encoder;

	public InputOutputChannel(
			IChannelManager manager, IServerConnection connection, IMessageParser parser, IMessageEncoder encoder)
	{
		super(manager, connection, parser);
		this.encoder = encoder;
	}

	protected void sendMessage(byte[] message)
	{
		try {
			connection.sendMessage(message);
			// System.out.println("sending: " + Arrays.toString(message));
			// System.out.println("sending: " + new String(message));

		} catch (ConnectionException e) {
			System.err.println("ConnectionChannel::sendMessage(): " + e);
			state.setLastErrorMessage(e.getMessage());
		}
	}

	@Override
	public void sendMessage(Map<String, IEffector> effectors)
	{
		byte[] message = encoder.encodeMessage(effectors);
		if (message != null) {
			sendMessage(message);
		}
	}
}
