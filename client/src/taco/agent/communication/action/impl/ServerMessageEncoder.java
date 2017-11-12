package taco.agent.communication.action.impl;

import com.google.gson.GsonBuilder;
import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.agent.communication.action.IMessageEncoder;
import java.util.Map;

public class ServerMessageEncoder implements IMessageEncoder
{
	@Override
	public byte[] encodeMessage(Map<String, IEffector> effectors)
	{
		return new GsonBuilder().setPrettyPrinting().create().toJson(new ActionMessage(effectors)).getBytes();
	}
}
