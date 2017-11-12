package taco.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.IPerceptor;
import taco.agent.communication.perception.IAudiCupPerceptor;

public abstract class AudiCupPerceptor implements IPerceptor, IAudiCupPerceptor
{
	private transient String name;

	/** The time of the measurement. */
	protected long timestamp;

	public AudiCupPerceptor(String name, long timestamp)
	{
		this.name = name;
		this.timestamp = timestamp;
	}

	@Override
	public String getName()
	{
		return name;
	}

	public void setName(String name)
	{
		this.name = name;
	}

	@Override
	public long getTimeStamp()
	{
		return timestamp;
	}
}
