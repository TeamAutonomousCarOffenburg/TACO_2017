package taco.agent.communication.action.impl;

import java.util.HashMap;
import java.util.Map;

import hso.autonomy.agent.communication.action.IActionPerformer;
import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.agent.communication.action.impl.Action;
import taco.agent.communication.action.IAudiCupAction;

public class AudiCupAction extends Action implements IAudiCupAction
{
	public AudiCupAction(IActionPerformer actionPerformer)
	{
		super(actionPerformer);
	}

	public void init(Map<String, IEffector> effectors)
	{
		this.effectors = effectors;
		actionEffectors = new HashMap<>(effectors);
	}

	@Override
	public IEffector getEffector(String name)
	{
		return effectors.get(name);
	}

	@Override
	public void setEffectors(Map<String, IEffector> effectors)
	{
		this.effectors = effectors;
		this.actionEffectors = effectors;
	}
}
