package taco.util.misc;

import java.util.LinkedList;
import java.util.List;

public class Distribution
{
	private int size;

	double mean;

	double variance;

	private List<Double> values;

	public Distribution(int size)
	{
		this.size = size;
		values = new LinkedList<>();
		mean = 0;
		variance = 0;
	}

	public void addValue(double value)
	{
		values.add(value);

		// remove first element if size is too big
		if (values.size() > size) {
			values.remove(0);
		}

		// Calculate mean
		mean = 0;
		for (double val : values) {
			mean += val;
		}
		mean /= values.size();

		// Calculate variance
		variance = 0;
		double dev;
		for (double val : values) {
			dev = (val - mean);
			variance += dev * dev;
		}
	}

	public double getMean()
	{
		return mean;
	}

	public double getVariance()
	{
		return variance;
	}

	public double getStandardDeviation()
	{
		return Math.sqrt(variance);
	}

	public boolean isValid()
	{
		return values.size() == size;
	}
}
