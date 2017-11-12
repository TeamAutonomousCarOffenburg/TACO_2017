/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.decision.behavior.basic;

import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;

/**
 * This is a behavior which does nothing. It is just an empty behavior if you
 * need one somewhere.
 * @author Ingo Schindler
 */
public class None extends Behavior
{
	public None(IThoughtModel thoughtModel)
	{
		super(IBehavior.NONE, thoughtModel);
	}
}
