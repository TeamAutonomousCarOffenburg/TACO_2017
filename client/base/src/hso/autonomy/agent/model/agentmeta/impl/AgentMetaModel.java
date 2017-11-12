/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta.impl;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmeta.IBodyPartConfiguration;
import hso.autonomy.agent.model.agentmeta.ICompositeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IHingeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.ISensorConfiguration;
import hso.autonomy.util.geometry.IPose3D;

/**
 * Agent meta model class
 *
 * @author Stefan Glaser
 */
public abstract class AgentMetaModel implements IAgentMetaModel
{
	protected final String modelName;

	protected List<IBodyPartConfiguration> bodyPartConfig;

	protected final String bodyPartContainingCamera;

	protected final IPose3D cameraOffset;

	private final float height;

	protected Collection<ISensorConfiguration> jointConfigs;

	protected String[] jointNames;

	protected String[] effectorNames;

	/**
	 * Constructor.
	 *
	 * @param modelName - the name of this model
	 * @param bodyPartContainingCamera - the name of the body part containing the
	 *        camera
	 */
	public AgentMetaModel(String modelName, String bodyPartContainingCamera, IPose3D cameraOffset, float height)
	{
		this.modelName = modelName;
		this.bodyPartContainingCamera = bodyPartContainingCamera;
		this.cameraOffset = cameraOffset;
		this.height = height;
		bodyPartConfig = createBodyPartConfigs();
	}

	protected abstract List<IBodyPartConfiguration> createBodyPartConfigs();

	@Override
	public String getName()
	{
		return modelName;
	}

	@Override
	public List<IBodyPartConfiguration> getBodyPartConfigurations()
	{
		return bodyPartConfig;
	}

	@Override
	public String getNameOfCameraBodyPart()
	{
		return bodyPartContainingCamera;
	}

	@Override
	public IPose3D getCameraOffset()
	{
		return cameraOffset;
	}

	@Override
	public List<IBodyPartConfiguration> getChildBodyConfigurations(IBodyPartConfiguration bodyPart)
	{
		List<IBodyPartConfiguration> childBodies = new ArrayList<>();

		if (bodyPart != null) {
			String parentName = bodyPart.getName();

			for (int i = 0; i < bodyPartConfig.size(); i++) {
				if (parentName.equals(bodyPartConfig.get(i).getParent())) {
					childBodies.add(bodyPartConfig.get(i));
				}
			}
		}

		return childBodies;
	}

	@Override
	public IBodyPartConfiguration getRootBodyConfiguration()
	{
		for (int i = 0; i < bodyPartConfig.size(); i++) {
			if (bodyPartConfig.get(i).getParent() == null) {
				return bodyPartConfig.get(i);
			}
		}

		return null;
	}

	@Override
	public Collection<ISensorConfiguration> getAvailableJoints()
	{
		if (jointConfigs == null) {
			ArrayList<ISensorConfiguration> jointConfigsList = new ArrayList<>();

			// Extract joints
			ISensorConfiguration jointConfig;
			for (IBodyPartConfiguration config : bodyPartConfig) {
				jointConfig = config.getJointConfiguration();
				if (jointConfig != null) {
					if (jointConfig instanceof ICompositeJointConfiguration) {
						Collections.addAll(
								jointConfigsList, ((ICompositeJointConfiguration) jointConfig).getHjConfigurations());
					} else {
						jointConfigsList.add(jointConfig);
					}
				}
			}
			jointConfigs = Collections.unmodifiableCollection(jointConfigsList);

			jointNames = new String[jointConfigsList.size()];
			effectorNames = new String[jointConfigsList.size()];
			for (int i = 0; i < jointConfigsList.size(); i++) {
				ISensorConfiguration config = jointConfigsList.get(i);
				jointNames[i] = config.getName();
				effectorNames[i] = ((IHingeJointConfiguration) config).getEffectorName();
			}
		}

		return jointConfigs;
	}

	@Override
	public String[] getAvailableJointNames()
	{
		if (jointNames == null) {
			getAvailableJoints();
		}

		return jointNames;
	}

	@Override
	public String[] getAvailableEffectorNames()
	{
		if (effectorNames == null) {
			getAvailableJoints();
		}

		return effectorNames;
	}

	@Override
	public float getHeight()
	{
		return height;
	}
}
