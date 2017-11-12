/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.decision.behavior;

import java.util.HashMap;
import java.util.Map;

public class BehaviorMapBase<T extends IBehavior>
{
	private Map<String, T> behaviors = new HashMap<>();

	public T get(String name)
	{
		return behaviors.get(name);
	}

	public T put(T behavior)
	{
		behaviors.put(behavior.getName(), behavior);
		return behavior;
	}

	public Map<String, T> getMap()
	{
		return behaviors;
	}
}