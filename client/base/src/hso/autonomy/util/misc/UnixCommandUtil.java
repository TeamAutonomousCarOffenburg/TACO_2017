/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Field;
import java.util.List;

/**
 * A collection of command line utilities for direct unix commands.
 * @author kdorer
 */
public class UnixCommandUtil
{
	/**
	 * Retrieves the pid of a unix process. (Possibly in Java 1.8 this will be
	 * added to the API of process??).
	 * @param process the process object to get the pid from
	 * @return the pid, -1 if not able to get it
	 * @throws NoSuchFieldException
	 * @throws SecurityException
	 * @throws IllegalArgumentException
	 * @throws IllegalAccessException
	 */
	public static int getPID(Process process)
			throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException
	{
		if (process.getClass().getName().equals("java.lang.UNIXProcess")) {
			/* get the PID on unix/linux systems */
			Field f = process.getClass().getDeclaredField("pid");
			f.setAccessible(true);
			return f.getInt(process);
		} else {
			return -1;
		}
	}

	/**
	 * Checks if the process with pid passed contains the passed name when run
	 * with ps. If so it is killed.
	 * @param pid the pid of the process to kill.
	 * @param name the name that has to appear when calling ps
	 * @return true if the process was killed.
	 */
	public static boolean killProcessConditional(String pid, String name)
	{
		String command = "ps " + pid;
		Process ps;
		try {
			ps = Runtime.getRuntime().exec(command);
			if (UnixCommandUtil.checkForOutput(ps, name)) {
				Runtime.getRuntime().exec("kill -9 " + pid);
				return true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return false;
	}

	/**
	 * Kills all processes with a given name. Use with care!
	 * @param name the name that has to appear when calling ps
	 */
	public static boolean killAll(String name)
	{
		try {
			Runtime.getRuntime().exec("killall -9 " + name);
			return true;
		} catch (IOException e) {
			e.printStackTrace();
		}
		return false;
	}

	/**
	 * Checks if the passed string is part of the output of the passed process.
	 * @param ps the process to get output from
	 * @param text the text that we expect to be part of the output
	 */
	public static boolean checkForOutput(Process ps, String text)
	{
		BufferedReader reader = new BufferedReader(new InputStreamReader(ps.getInputStream()));
		String line;
		boolean found = false;
		try {
			line = reader.readLine();
			while (line != null) {
				if (line.contains(text)) {
					found = true;
					break;
				}
				line = reader.readLine();
			}
			reader.close();
			return found;

		} catch (IOException e) {
			e.printStackTrace();
		}
		return false;
	}

	public static Process launch(String command, String[] params, File path)
	{
		try {
			return Runtime.getRuntime().exec(command, params, path);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}

	public static Process launchAndConsume(String... command) throws IOException
	{
		ProcessBuilder builder = new ProcessBuilder(command);
		builder.redirectOutput(new File("/dev/null"));
		builder.redirectError(new File("/dev/null"));
		return builder.start();
	}

	public static Process launchAndConsume(List<String> command) throws IOException
	{
		return launchAndConsume(command.toArray(new String[0]));
	}
}
