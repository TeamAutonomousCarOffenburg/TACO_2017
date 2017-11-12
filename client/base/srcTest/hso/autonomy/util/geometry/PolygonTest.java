package hso.autonomy.util.geometry;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.junit.Test;
import static junit.framework.Assert.assertTrue;

public class PolygonTest
{
	@Test
	public void testContains()
	{
		double x1 = 0.4;
		double y1 = 1.7;
		double x2 = 0.98;
		double y2 = 1.96;

		double px = 0.69;
		double py = 1.83;

		Polygon polygon =
				new Polygon(new Vector2D(x1, y1), new Vector2D(x2, y1), new Vector2D(x2, y2), new Vector2D(x1, y2));
		Vector2D point = new Vector2D(px, py);
		assertTrue(polygon.contains(point));
	}
}
