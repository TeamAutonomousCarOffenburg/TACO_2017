/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * En- and Decoder of messages
 * @author dorer
 */
public class SayCoder
{
	/**
	 * @param id a number (0..25) to encode
	 * @return a String representation of the number
	 */
	public static String encodeID(int id)
	{
		return "" + (char) ('a' + id);
	}

	/**
	 * @param text the text to decode
	 * @return a number (0..25) decoded from text
	 */
	public static int decodeID(String text)
	{
		if (text.length() != 1) {
			return -1;
		}
		return text.charAt(0) - 'a';
	}

	/**
	 * @param text the text to decode
	 * @return a number (0..25) decoded from text
	 */
	public static int decodeID(char text)
	{
		return text - 'a';
	}

	/**
	 * Encodes the passed vectors x and y coordinate into a String
	 * @return the passed vectors x and y coordinate encoded into a String
	 */
	public static String encodePosition(Vector3D position)
	{
		int xShort = (int) ((position.getX() + 20) * 10);
		int yShort = (int) ((position.getY() + 20) * 10);
		return encodeTwoDigits(xShort) + encodeTwoDigits(yShort);
	}

	/**
	 * Converts a position string into a Vector3D
	 * @param text the text to convert as 6 digit String (see encodePosition)
	 * @return the decoded position, null if the text did not allow to decode
	 */
	public static Vector3D decodePosition(String text)
	{
		if (text.length() != getPositionLength()) {
			return null;
		}
		if (text.equals(encodeInvalidPosition())) {
			return null;
		}
		try {
			String xString = text.substring(0, 2);
			String yString = text.substring(2, 4);
			int x = decodeTwoDigits(xString);
			int y = decodeTwoDigits(yString);

			float xFloat = (x / 10.0f) - 20;
			float yFloat = (y / 10.0f) - 20;
			return new Vector3D(xFloat, yFloat, 0);

		} catch (Exception e) {
			return null;
		}
	}

	/**
	 * @return the encoding of an invalid position
	 */
	public static String encodeInvalidPosition()
	{
		return "aaaa";
	}

	/**
	 * @return the string length of a position encoding
	 */
	public static int getPositionLength()
	{
		return 4;
	}

	/**
	 * @param name the name to encode
	 * @return the encoded version of the passed name
	 */
	public static String encodeName(String name)
	{
		return encodeID(Math.abs(name.hashCode()) % 20);
	}

	/**
	 * @param digits a number (0..25) to encode
	 * @return a String representation of the number
	 */
	public static String encodeTwoDigits(int digits)
	{
		int lowDigit = digits % 26;
		int highDigit = digits / 26;
		return encodeID(highDigit) + encodeID(lowDigit);
	}

	/**
	 * @param digits a String representation of the number
	 * @return the decoded number
	 */
	public static int decodeTwoDigits(String digits)
	{
		if (digits.length() != 2) {
			return 0;
		}
		return decodeID(digits.charAt(0)) * 26 + decodeID(digits.charAt(1));
	}
}
