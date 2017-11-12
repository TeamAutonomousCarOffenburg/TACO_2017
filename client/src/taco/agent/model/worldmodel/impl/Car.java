package taco.agent.model.worldmodel.impl;

import hso.autonomy.agent.model.worldmodel.impl.MovableObject;
import taco.agent.model.worldmodel.ICar;

public class Car extends MovableObject implements ICar
{
	protected Car(String name, float cycleTime)
	{
		super(name, cycleTime);
	}
}
