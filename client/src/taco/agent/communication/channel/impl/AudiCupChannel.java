package taco.agent.communication.channel.impl;

import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.IChannelState.ConnectionState;
import hso.autonomy.agent.communication.channel.impl.InputOutputChannel;
import hso.autonomy.util.connection.ConnectionException;
import hso.autonomy.util.connection.impl.ServerConnection;
import taco.agent.communication.action.impl.ServerMessageEncoder;
import taco.agent.communication.perception.impl.ServerMessageParser;

public class AudiCupChannel extends InputOutputChannel
{
	private static final int PORT = 63236;

	public AudiCupChannel(IChannelManager manager, String host)
	{
		super(manager, new ServerConnection(host, PORT), new ServerMessageParser(), new ServerMessageEncoder());
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
			getManager().lostConnection(AudiCupChannel.this);
		});
		ourThread.start();
		return true;
	}
}
