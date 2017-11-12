#pragma once

class ByteConverter
{
  public:
	union bytes_int
	{
		unsigned char c[4];
		int i;
	};

	static void IntToBytes(int in, unsigned char *bytes)
	{
		union bytes_int ic;
		ic.i = in;
		for (int i = 0; i < 4; i++) {
#ifdef __BYTE_ORDER__
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			bytes[i] = ic.c[3 - i];
#else
			bytes[i] = ic.c[i];
#endif
#else
/** \todo find out Endianness for other compiles */
#error "Macro „__BYTE_ORDER__“ is not defined."
#endif
		}
	}

	static int BytesToInt(unsigned char *bytes)
	{
		union bytes_int ic;
		for (int i = 0; i < 4; i++) {
#ifdef __BYTE_ORDER__
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			ic.c[3 - i] = bytes[i];
#else
			ic.c[i] = bytes[i];
#endif
#else
/** \todo find out Endianness for other compiles */
#error "Macro „__BYTE_ORDER__“ is not defined."
#endif
		}
		return ic.i;
	}
};
