package taco.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.IMessageParser;
import hso.autonomy.agent.communication.perception.IPerceptor;
import hso.autonomy.agent.communication.perception.PerceptorConversionException;
import hso.autonomy.util.misc.GsonUtil;
import java.util.Map;

public class ServerMessageParser implements IMessageParser
{
	@Override
	public Map<String, IPerceptor> parseMessage(byte[] message) throws PerceptorConversionException
	{
		PerceptionMessage perception = GsonUtil.fromJson(new String(message), PerceptionMessage.class);
		return perception.toPerceptorMap();
	}

	@Override
	public String getErrorString(byte[] message)
	{
		return new String(message);
	}
}
