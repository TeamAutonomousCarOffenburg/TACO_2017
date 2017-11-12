/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.channel;

/**
 * @author rschilli
 */
public interface IChannelState {
	enum ConnectionState
	{
		NOT_CONNECTED,
		TRYING_TO_CONNECT,
		CONNECTED,
		DISCONNECTED,
		CONNECTION_FAILED
	}

	enum DataState
	{
		CORRECT_DATA,
		INVALID_DATA,
		NO_DATA
	}

	boolean isConnected();

	ConnectionState getConnectionState();

	DataState getDataState();

	void setLastErrorMessage(String lastErrorMessage);

	void setConnectionState(ConnectionState connectionState);

	void setDataState(DataState dataState);
}
