package taco;

import hso.autonomy.util.commandline.BooleanArgument;
import hso.autonomy.util.commandline.EnumArgument;
import hso.autonomy.util.commandline.HelpArgument;
import hso.autonomy.util.commandline.StringArgument;
import taco.agent.agentruntime.AudiCupAgentRuntime;
import taco.agent.agentruntime.AudiCupAgentRuntimeParameters;
import taco.agent.agentruntime.ComponentFactory;
import taco.agent.agentruntime.scenarios.IScenario;
import taco.agent.agentruntime.scenarios.Scenario;
import taco.agent.model.agentmeta.impl.CarMetaModel;
import taco.agent.model.agentmeta.impl.CarMetaModelVersion;

public class AudiCupClient
{
	public static void main(String[] args)
	{
		StringArgument serverArgument = new StringArgument("server", "localhost", "server IP");
		StringArgument metaModelDirectoryArgument = new StringArgument("metaModelDirectory",
				CarMetaModel.DEFAULT_DIRECTORY, "The directory containing the meta model JSON files");
		EnumArgument<CarMetaModelVersion> metaModelArgument = new EnumArgument<>(
				"metaModel", CarMetaModelVersion.DEFAULT, "The car meta model to use", CarMetaModelVersion.class);
		EnumArgument<Scenario> scenarioArgument =
				new EnumArgument<>("scenario", Scenario.DEFAULT, "The scenario to use", Scenario.class);
		BooleanArgument logArgument = new BooleanArgument("log", "write perceptions into a log file");
		new HelpArgument(serverArgument, metaModelArgument, metaModelDirectoryArgument, scenarioArgument, logArgument)
				.parse(args);

		String server = serverArgument.parse(args);
		String metaModelDirectory = metaModelDirectoryArgument.parse(args);
		CarMetaModelVersion metaModelVersion = metaModelArgument.parse(args);
		IScenario scenario = scenarioArgument.parse(args).construct();
		boolean log = logArgument.parse(args);

		AudiCupAgentRuntimeParameters runtimeParameters = new AudiCupAgentRuntimeParameters(
				new ComponentFactory(), server, metaModelDirectory, metaModelVersion, scenario, log, null);
		AudiCupAgentRuntime client = new AudiCupAgentRuntime(runtimeParameters);
		client.startClient();
	}
}
