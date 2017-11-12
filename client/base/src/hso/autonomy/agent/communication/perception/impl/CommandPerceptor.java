/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.ICommandPerceptor;

/**
 * Remote Command Perceptor
 *
 * @author Klaus Dorer
 */
public class CommandPerceptor extends Perceptor implements ICommandPerceptor
{
	private String command;

	/**
	 * Assignment constructor
	 *
	 * @param command the remote command
	 */
	public CommandPerceptor(String command)
	{
		super("command");
		this.command = command;
	}

	@Override
	public String getCommand()
	{
		return command;
	}
}
