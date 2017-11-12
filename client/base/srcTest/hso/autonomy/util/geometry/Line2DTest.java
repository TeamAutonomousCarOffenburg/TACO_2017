package hso.autonomy.util.geometry;

import org.junit.Test;

import static junit.framework.Assert.assertEquals;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Line2DTest
{
	@Test
	public void getClosestPoseTest()
	{
		Line2D testee = new Line2D(0, 0, Angle.rad(0), 10);
		Pose2D result = testee.getClosestPose(new Vector2D(1, 1));

		assertEquals(result.getPosition().getX(), 1, 0.000001);
		assertEquals(result.getPosition().getY(), 0, 0.000001);
		assertEquals(result.getAngle().radians(), 0, 0.000001);

		result = testee.getClosestPose(new Vector2D(2, 2));
		assertEquals(result.getPosition().getX(), 2, 0.000001);
		assertEquals(result.getPosition().getY(), 0, 0.000001);
		assertEquals(result.getAngle().radians(), 0, 0.000001);

		Line2D testee2 = new Line2D(-1.0, -1.0, Angle.rad(Math.PI / 4), 10.0);
		result = testee2.getClosestPose(new Vector2D(2, 0));
		assertEquals(result.getPosition().getX(), 1, 0.000001);
		assertEquals(result.getPosition().getY(), 1, 0.000001);
		assertEquals(result.getAngle().radians(), Math.PI * 0.25, 0.000001);

		result = testee2.getClosestPose(new Vector2D(0, 0));
		assertEquals(result.getPosition().getX(), 0, 0.000001);
		assertEquals(result.getPosition().getY(), 0, 0.000001);
		assertEquals(result.getAngle().radians(), Math.PI * 0.25, 0.000001);
	};

	@Test
	public void getTrailTest()
	{
		Line2D testee = new Line2D(1, 1, Angle.deg(-45));
		List<Vector2D> points = testee.getTrail(new Vector2D(1, 1), new Vector2D(3, -1.1));
		//		     for(Vector2d point : points)
		//   {
		//		     std::cout << "point x: " << point(0) << " y: " << point(1) << endl;
		//   }
		assertEquals(points.size(), 10, 0.001);
		assertEquals(points.get(0).getX(), 1.205, 0.001);
		assertEquals(points.get(4).getY(), -0.025, 0.0001);
		assertEquals(points.get(9).getX(), 3.05, 0.0001);
	}
}
