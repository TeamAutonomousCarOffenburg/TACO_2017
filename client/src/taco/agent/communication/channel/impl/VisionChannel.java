package taco.agent.communication.channel.impl;

import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.IChannelState.ConnectionState;
import hso.autonomy.agent.communication.channel.impl.InputChannel;
import hso.autonomy.util.connection.ConnectionException;
import hso.autonomy.util.connection.impl.ServerConnection;
import taco.agent.communication.perception.impl.ServerMessageParser;

public class VisionChannel extends InputChannel
{
	private static final int PORT = 63237;

	public VisionChannel(IChannelManager manager, String host)
	{
		super(manager, new ServerConnection(host, PORT, false), new ServerMessageParser());
	}

	@Override
	protected boolean startReceiveLoop()
	{
		Thread ourThread = new Thread(() -> {
			try {
				connection.startReceiveLoop();
				state.setConnectionState(ConnectionState.NOT_CONNECTED);
			} catch (ConnectionException e) {
				state.setConnectionState(ConnectionState.DISCONNECTED);
			} catch (RuntimeException e) {
				state.setConnectionState(ConnectionState.DISCONNECTED);
				e.printStackTrace();
			}
			getManager().lostConnection(VisionChannel.this);
		});
		ourThread.start();
		return true;
	}
}
