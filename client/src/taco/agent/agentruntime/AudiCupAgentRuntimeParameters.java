package taco.agent.agentruntime;

import hso.autonomy.util.geometry.IPose2D;
import kdo.util.parameter.ParameterMap;
import taco.agent.agentruntime.scenarios.IScenario;
import taco.agent.model.agentmeta.impl.CarMetaModelVersion;

public class AudiCupAgentRuntimeParameters
{
	private final ComponentFactory factory;

	private final String server;

	private final String metaModelDirectory;

	private final CarMetaModelVersion version;

	private final IScenario scenario;

	private final boolean log;

	/** the global pose, where the car starts, null if we should take it from the scenario */
	private final IPose2D startPose;

	private final ParameterMap parameterMap;

	public AudiCupAgentRuntimeParameters(ComponentFactory factory, String server, String metaModelDirectory,
			CarMetaModelVersion version, IScenario scenario, boolean log, IPose2D startPose, ParameterMap parameterMap)
	{
		this.factory = factory;
		this.server = server;
		this.metaModelDirectory = metaModelDirectory;
		this.version = version;
		this.scenario = scenario;
		this.log = log;
		this.startPose = startPose;
		if (parameterMap == null) {
			parameterMap = new ParameterMap();
		}
		this.parameterMap = parameterMap;
	}

	public ComponentFactory getFactory()
	{
		return factory;
	}

	public String getServer()
	{
		return server;
	}

	public String getMetaModelDirectory()
	{
		return metaModelDirectory;
	}

	public CarMetaModelVersion getVersion()
	{
		return version;
	}

	public IScenario getScenario()
	{
		return scenario;
	}

	public boolean isLog()
	{
		return log;
	}

	public IPose2D getStartPose()
	{
		return startPose;
	}

	public ParameterMap getParameterMap()
	{
		return parameterMap;
	}
}