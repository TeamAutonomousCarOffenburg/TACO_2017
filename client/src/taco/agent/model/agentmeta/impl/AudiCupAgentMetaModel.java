package taco.agent.model.agentmeta.impl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonDeserializer;

import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.agent.model.agentmeta.IBodyPartConfiguration;
import hso.autonomy.agent.model.agentmeta.impl.AgentMetaModel;
import hso.autonomy.agent.model.agentmeta.impl.BodyPartConfiguration;
import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;
import hso.autonomy.util.file.FileUtil;
import hso.autonomy.util.geometry.Pose3D;

public class AudiCupAgentMetaModel extends AgentMetaModel
{
	public static final String NAME = "AudiCup";

	private static Gson gson =
			new GsonBuilder()
					.registerTypeAdapter(Rotation.class,
							(JsonDeserializer<Rotation>) (src, typeOfSrc, context) -> {
								double angle = src.getAsJsonObject().getAsJsonPrimitive("angle").getAsDouble();
								Vector3D axis = context.deserialize(src.getAsJsonObject().get("axis"), Vector3D.class);
								return new Rotation(axis, Math.toRadians(angle));
							})
					.create();

	private final CarMetaModel carMetaModel;

	public AudiCupAgentMetaModel(String metaModelDirectory, CarMetaModelVersion version)
	{
		super("Audi", "torso", new Pose3D(Vector3D.ZERO, Rotation.IDENTITY), 0);

		String json = FileUtil.readFile(metaModelDirectory + "/" + version.fileName);
		carMetaModel = gson.fromJson(json, CarMetaModel.class);
	}

	@Override
	public Map<String, IEffector> createEffectors()
	{
		// not needed here
		return new HashMap<>();
	}

	@Override
	protected List<IBodyPartConfiguration> createBodyPartConfigs()
	{
		ArrayList<IBodyPartConfiguration> bodyPartConfigurations = new ArrayList<>();

		bodyPartConfigurations.add(new BodyPartConfiguration("torso", null, Vector3D.ZERO, 0, Vector3D.ZERO,
				new SensorConfiguration(null, null), Vector3D.ZERO, new SensorConfiguration(null, null),
				new SensorConfiguration(null, null), new SensorConfiguration(null, null),
				new SensorConfiguration(null, null)));

		return bodyPartConfigurations;
	}

	public CarMetaModel getCarMetaModel()
	{
		return carMetaModel;
	}
}
