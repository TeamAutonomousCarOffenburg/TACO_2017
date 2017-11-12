package taco.agent.communication.perception;

import com.google.gson.annotations.SerializedName;

import hso.autonomy.util.geometry.Area2D;

public class RecognizedObject
{
	@SerializedName("class_id")
	private RecognizedObjectType type;

	@SerializedName("score")
	private double confidence;

	@SerializedName("roi")
	private Area2D.Int area;

	/** only used by simulator. If this is true, area contains the world coordinates (scaled by 100) */
	private boolean inCarCoordinates;

	public RecognizedObject(RecognizedObjectType type, Area2D.Int area, double confidence)
	{
		this.type = type;
		this.area = area;
		this.confidence = confidence;
		inCarCoordinates = true;
	}

	public RecognizedObjectType getType()
	{
		return type;
	}

	public double getConfidence()
	{
		return confidence;
	}

	public Area2D.Int getArea()
	{
		return area;
	}

	public boolean isInCarCoordinates()
	{
		return inCarCoordinates;
	}

	@Override
	public String toString()
	{
		return "\nType: " + type + " , confidence: " + confidence + " , area: " + area.toString();
	}
}
