package taco.agent.model.thoughtmodel.impl;

import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;

public abstract class Detection
{
	/** true if this detection is valid */
	protected boolean valid;

	/** time since this detection is valid. Use only if valid is true */
	protected float validSince;

	/** time since this detection is invalid. Use only if valid is false */
	protected float invalidSince;

	/** the number of consecutive true detections before valid is switched to true */
	private int consecutiveTrueValues;

	/** the number of consecutive false detections before valid is switched to false */
	private int consecutiveFalseValues;

	/** the number of times the opposite detection is detected in a row */
	private int consecutiveCount;

	public Detection()
	{
		this(1, 1);
	}

	public Detection(int consecutiveTrueValues, int consecutiveFalseValues)
	{
		this.consecutiveTrueValues = consecutiveTrueValues;
		this.consecutiveFalseValues = consecutiveFalseValues;
		valid = false;
		validSince = 0;
		invalidSince = 0;
		consecutiveCount = consecutiveTrueValues;
	}

	public abstract void update(IAudiCupThoughtModel thoughtModel);

	public boolean isValid()
	{
		return valid;
	}

	public float getValidSince()
	{
		return validSince;
	}

	public float getValidityTime(float now)
	{
		return now - validSince;
	}

	public float getInvalidSince()
	{
		return invalidSince;
	}

	public float getInValidityTime(float now)
	{
		return now - invalidSince;
	}

	protected void setValidity(boolean newValidity, float now)
	{
		if (newValidity == true) {
			if (!valid) {
				consecutiveCount++;
				if (consecutiveCount >= consecutiveTrueValues) {
					validSince = now;
					valid = true;
					consecutiveCount = 0;
				}
			} else {
				consecutiveCount = 0;
			}
		} else {
			if (valid) {
				consecutiveCount++;
				if (consecutiveCount >= consecutiveFalseValues) {
					invalidSince = now;
					valid = false;
					consecutiveCount = 0;
				}
			} else {
				consecutiveCount = 0;
			}
		}
	}
}