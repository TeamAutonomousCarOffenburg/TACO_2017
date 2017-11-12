/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry.positionFilter;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 *
 * @author kdorer
 */
public class PositionKalmanFilter extends BaseFilter
{
	private KalmanFilter filter;

	private boolean shouldReset;

	public PositionKalmanFilter()
	{
		init(Vector3D.ZERO);
	}

	private void init(Vector3D realPos)
	{
		// A = state transition matrix for state {x, y, z}
		RealMatrix A = new Array2DRowRealMatrix(new double[][] {
				{1, 0, 0}, //
				{0, 1, 0}, //
				{0, 0, 1}, //
		});
		// B = control input matrix
		RealMatrix B = new Array2DRowRealMatrix(new double[][] {
				{1, 0, 0}, //
				{0, 1, 0}, //
				{0, 0, 1}, //
		});

		// H = measurement matrix
		RealMatrix H = new Array2DRowRealMatrix(new double[][] {
				{1, 0, 0}, //
				{0, 1, 0}, //
				{0, 0, 1}, //
		});
		// x = initial guess
		double[] pos = new double[] {realPos.getX(), realPos.getY(), realPos.getZ()};
		RealVector x = new ArrayRealVector(pos);

		// Q = estimate of process error
		double processNoise = 0.001;
		RealMatrix Q = new Array2DRowRealMatrix(new double[][] {
				{processNoise, 0, 0}, //
				{0, processNoise, 0}, //
				{0, 0, processNoise}, //
		});

		// P0 = initial guess of covariance matrix
		RealMatrix P0 = new Array2DRowRealMatrix(new double[][] {
				//
				{1, 0, 0}, //
				{0, 1, 0}, //
				{0, 0, 1}, //
		});
		// R = estimate of measurement noise
		// position measurement noise (meter)
		double mnoise = 0.1;
		RealMatrix R = new Array2DRowRealMatrix(new double[][] {
				{mnoise, 0, 0}, //
				{0, mnoise, 0}, //
				{0, 0, mnoise}, //
		});

		ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
		MeasurementModel mm = new DefaultMeasurementModel(H, R);
		filter = new KalmanFilter(pm, mm);
	}

	public void predict()
	{
		// here we could add our actions with a vector u
		filter.predict();
	}

	@Override
	public Vector3D filterPosition(Vector3D newPosition, Vector3D oldPosition, Vector3D speed)
	{
		if (shouldReset) {
			// in case of too far new pos, we believe in new
			shouldReset = false;
			init(newPosition);
			// System.out.println("init" + newPosition + " old: " + oldPosition
			// + " d: " + Vector3D.distance(newPosition, oldPosition));
			return newPosition;
		}

		// here we could add our actions with a vector u
		// we have to take factor3 since this function is called per vision cycle
		double[] u = {speed.getX() * 3, speed.getY() * 3, speed.getZ() * 3};
		filter.predict(u);

		RealVector z = new ArrayRealVector(new double[] {newPosition.getX(), newPosition.getY(), newPosition.getZ()});

		filter.correct(z);

		Vector3D result = new Vector3D(
				filter.getStateEstimation()[0], filter.getStateEstimation()[1], filter.getStateEstimation()[2]);

		// System.out.println(filter.toString());
		// System.out.println("Observed: " + position + " filtered: " + result);
		// System.out.println(filter.getErrorCovarianceMatrix().toString());
		return result;
	}

	@Override
	public void reset()
	{
		shouldReset = true;
	}
}
