/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.connection.impl;

import java.io.BufferedInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import hso.autonomy.util.commandline.BooleanArgument;
import hso.autonomy.util.commandline.HelpArgument;
import hso.autonomy.util.commandline.IntegerArgument;

/**
 *
 * @author kdorer
 */
public class TCPIPTestServer
{
	/** the port the server is listening to */
	private int portNumber;

	/** time to sleep after each message (in ms), -1 if no sleep */
	private int timeBetweenMessages;

	/** the number of messages to send, -1 if infinite */
	private long numberOfMessages;

	/** true if we should print messages */
	private boolean verbose;

	private DataOutputStream out;

	private BufferedInputStream in;

	private ServerSocket serverSocket;

	private Socket socket;

	/** set to true to end the thread */
	private boolean stopThread;

	private int countConnections;

	public TCPIPTestServer(int port, int timeBetween, long numberOfMessages, boolean verbose)
	{
		portNumber = port;
		timeBetweenMessages = timeBetween;
		this.numberOfMessages = numberOfMessages;
		this.verbose = verbose;
		socket = null;
		stopThread = false;
		countConnections = 0;
	}

	public boolean isConnected()
	{
		return socket != null;
	}

	public void connect()
	{
		Thread ourThread = new Thread(new AcceptThread());
		ourThread.start();
	}

	/*
	 * Close server connection
	 */
	public void closeConnection()
	{
		try {
			if (out != null) {
				out.close();
				out = null;
			}
			if (in != null) {
				in.close();
				in = null;
			}
			if (socket != null) {
				socket.close();
				socket = null;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		// System.out.println("Connection disconnected");
	}

	public void stopListening()
	{
		stopThread = true;
		closeConnection();
		try {
			if (!serverSocket.isClosed()) {
				serverSocket.close();
			}
		} catch (IOException e1) {
			e1.printStackTrace();
		}
	}

	public void sendTestMessages() throws InterruptedException
	{
		if (out == null) {
			System.out.println("No connection, can not send");
			return;
		}
		long start = System.nanoTime();
		long i = 0;
		while (numberOfMessages < 0 || i < numberOfMessages) {
			// sending
			if (verbose) {
				System.out.println("Sending message : " + i);
			}
			float timeSend = sendOneMessage(i);

			// receiving
			float timeReceive = receiveAnswer();

			long time = (long) (timeSend + timeReceive);
			long sleepTime = timeBetweenMessages - time;
			if (sleepTime > 0) {
				Thread.sleep(sleepTime);
			}
			i++;
		}
		long end = System.nanoTime();
		float time = (end - start) / 1000000f;
		System.out.println("Messages sent: " + numberOfMessages + " Overall time: " + time + " ms");
	}

	private float receiveAnswer()
	{
		long start = System.nanoTime();
		byte[] body = new byte[1348];
		try {
			in.read(body);
			TCPIPTestMessageCreator.printMessage(body, verbose);

		} catch (IOException e) {
			e.printStackTrace();
		}
		long end = System.nanoTime();
		float time = (end - start) / 1000000f;
		return time;
	}

	private float sendOneMessage(long count)
	{
		long start = System.nanoTime();
		byte[] body = TCPIPTestMessageCreator.createServerMessage(count);
		try {
			out.write(body);
			out.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
		long end = System.nanoTime();
		float time = (end - start) / 1000000f;
		// System.out.println("Sending took: " + time);
		return time;
	}

	private final class AcceptThread implements Runnable
	{
		@Override
		public void run()
		{
			try {
				serverSocket = new ServerSocket(portNumber);
				System.out.println("Waiting for connections");
				while (!stopThread) {
					Socket newSocket = serverSocket.accept();
					if (socket != null) {
						closeConnection();
					}
					socket = newSocket;
					in = new BufferedInputStream(socket.getInputStream());
					out = new DataOutputStream(socket.getOutputStream());
					socket.setTcpNoDelay(true);
					countConnections++;
					System.out.println("Have new connection: " + countConnections);
					try {
						sendTestMessages();
						stopListening();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			} catch (IOException e) {
				if (!e.getMessage().equals("socket closed")) {
					e.printStackTrace();
				}
			} catch (RuntimeException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Starts a TCPIP Server that sends 1348 bytes each 10 ms and waits for an
	 * answer of 456 bytes. Start the server with
	 * "java -cp magmaTestTCPIP.jar magma.util.connection.tcpip.TCPIPTestServer"
	 * Use --help for help on the available arguments.
	 */
	public static void main(String[] args) throws InterruptedException
	{
		IntegerArgument PORT =
				new IntegerArgument("port", 5790, 0, "the port on which the server is listening for incoming");
		IntegerArgument SLEEP_TIME = new IntegerArgument(
				"time", 10, -1, "the number of milliseconds to wait between two messages, if no wait");
		IntegerArgument MESSAGES =
				new IntegerArgument("messages", 6000, -1, "the number of messages to send, -1 if infinite");
		BooleanArgument QUIET = new BooleanArgument("quiet", "if the server should not print messages");

		new HelpArgument(PORT, SLEEP_TIME, MESSAGES, QUIET).parse(args);

		int port = PORT.parse(args);
		int time = SLEEP_TIME.parse(args);
		int messages = MESSAGES.parse(args);
		boolean quiet = QUIET.parse(args);

		TCPIPTestServer control = new TCPIPTestServer(port, time, messages, !quiet);
		control.connect();
	}
}
