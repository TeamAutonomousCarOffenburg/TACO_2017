package hso.autonomy.util.geometry;

import java.util.ArrayList;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

import static org.junit.Assert.assertTrue;

public class VectorUtilsTest
{
	@Test
	public void testAverage()
	{
		assertTrue(new Vector3D(1, 2, 3).equals(VectorUtils.average(new Vector3D(1, 2, 3))));
		assertTrue(
				new Vector3D(0.5, 2.5, 3.5).equals(VectorUtils.average(new Vector3D(-1, 2, 3), new Vector3D(2, 3, 4))));
	}

	@Test
	public void testAverageEmptyList()
	{
		assertTrue(Vector3D.ZERO.equals(VectorUtils.average(new ArrayList<>())));
		assertTrue(Vector3D.ZERO.equals(VectorUtils.average()));
	}
}
