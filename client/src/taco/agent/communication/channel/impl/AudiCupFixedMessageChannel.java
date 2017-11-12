package taco.agent.communication.channel.impl;

import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.IChannelState.ConnectionState;
import hso.autonomy.agent.communication.channel.impl.InputOutputChannel;
import hso.autonomy.util.connection.ConnectionException;
import hso.autonomy.util.connection.impl.TimedConnection;
import taco.agent.communication.action.impl.ServerMessageEncoder;
import taco.agent.communication.perception.impl.ServerMessageParser;

public class AudiCupFixedMessageChannel extends InputOutputChannel
{
	private String fixMessage;

	private float ticks = 0;

	public AudiCupFixedMessageChannel(IChannelManager manager)
	{
		super(manager, new TimedConnection(20), new ServerMessageParser(), new ServerMessageEncoder());

		fixMessage =
				"{"
				+ "\"CarIMU\":{\"gyro\":{\"q0\":1.0,\"q1\":0.0,\"q2\":0.0,\"q3\":0.0},"
				+ "\"acceleration\":{\"x\":0.47194504737854006,\"y\":-0.04658158868551254,\"z\":-10.46982479095459}},\n"
				+ "\"US_Front_Left\":{\"value\":0.4000000059604645},"
				+ "\"US_Front_Center_Left\":{\"value\":4.230000019073486},"
				+ "\"US_Front_Center\":{\"value\":4.230000019073486},"
				+ "\"US_Front_Center_Right\":{\"value\":4.230000019073486},"
				+ "\"US_Front_Right\":{\"value\":0.5799999833106995},"
				+ "\"US_Side_Left\":{\"value\":0.15000000596046449},"
				+ "\"US_Side_Right\":{\"value\":1.2100000381469727},"
				+ "\"US_Rear_Left\":{\"value\":0.9300000071525574},"
				+ "\"US_Rear_Center\":{\"value\":1.7000000476837159},"
				+ "\"US_Rear_Right\":{\"value\":4.320000171661377},"
				+ "\"WH_WheelSpeed_Sensor_Right\":{\"ticks\":0,\"direction\":1},"
				+ "\"WH_WheelSpeed_Sensor_Left\":{\"ticks\":0,\"direction\":1}}";
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
			getManager().lostConnection(AudiCupFixedMessageChannel.this);
		});
		ourThread.start();
		return true;
	}

	@Override
	public void update(byte[] message)
	{
		// create some slow forward movement
		ticks += 0.2;
		String updatedMessage = fixMessage.replaceAll("ticks\":0", "ticks\":" + (int) ticks);
		super.update(updatedMessage.getBytes());
	}
}
