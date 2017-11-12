/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.file;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;

/**
 * Utils for serialization tests
 * @author Klaus Dorer
 */
public class SerializationUtil
{
	/**
	 * Serializes the passed object to a byte array and deserializes it again
	 * @param testee the object to serialize
	 * @return the double serialized object
	 * @throws IOException should never happen
	 * @throws ClassNotFoundException if a class may not be desirialized
	 */
	public static Object doubleSerialize(Object testee) throws IOException, ClassNotFoundException
	{
		// write object to byte array (memory)
		ByteArrayOutputStream byteout = new ByteArrayOutputStream();
		ObjectOutputStream oos = new ObjectOutputStream(byteout);
		oos.writeObject(testee);

		// read object from byte array (memory)
		ByteArrayInputStream bytein = new ByteArrayInputStream(byteout.toByteArray());
		ObjectInputStream ois = new ObjectInputStream(bytein);
		return ois.readObject();
	}

	public static byte[] convertToBytes(Object object) throws IOException
	{
		try (ByteArrayOutputStream bos = new ByteArrayOutputStream(); ObjectOutput out = new ObjectOutputStream(bos)) {
			out.writeObject(object);
			return bos.toByteArray();
		}
	}

	public static Object convertFromBytes(byte[] bytes) throws IOException, ClassNotFoundException
	{
		try (ByteArrayInputStream bis = new ByteArrayInputStream(bytes); ObjectInput in = new ObjectInputStream(bis)) {
			return in.readObject();
		}
	}
}
