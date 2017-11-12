package taco.agent.model.agentmeta.impl;

public enum CarMetaModelVersion {
	TACO_2016("CarMetaModel2016.json"),
	TACO_2017("CarMetaModel2017.json");

	public static final CarMetaModelVersion DEFAULT = TACO_2017;

	public final String fileName;

	CarMetaModelVersion(String fileName)
	{
		this.fileName = fileName;
	}
}
