/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

import java.util.Map;

/**
 * A Server Message Parser receives a String from some interface layer, parses
 * it into a list of Perceptors and returns them.
 *
 * @author Simon Raffeiner
 */
public interface IMessageParser {
	/**
	 * Parses a message into IPerceptor objects
	 * @param message the message to parse
	 * @return a map of IPerceptor objects created during parsing
	 * @throws PerceptorConversionException
	 */
	Map<String, IPerceptor> parseMessage(byte[] message) throws PerceptorConversionException;

	/**
	 * How to output <code>message</code> in case parsing fails.
	 */
	String getErrorString(byte[] message);
}
