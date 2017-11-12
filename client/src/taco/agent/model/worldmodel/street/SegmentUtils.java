package taco.agent.model.worldmodel.street;

import static taco.agent.model.worldmodel.street.SegmentType.CURVE_BIG;
import static taco.agent.model.worldmodel.street.SegmentType.CURVE_SMALL;
import static taco.agent.model.worldmodel.street.SegmentType.PARKING_SPACE_HORIZONTAL;
import static taco.agent.model.worldmodel.street.SegmentType.PARKING_SPACE_VERTICAL;
import static taco.agent.model.worldmodel.street.SegmentType.STRAIGHT;
import static taco.agent.model.worldmodel.street.SegmentType.STRAIGHT_WITH_CROSSWALK;
import static taco.agent.model.worldmodel.street.SegmentType.S_CURVE_BOTTOM;
import static taco.agent.model.worldmodel.street.SegmentType.S_CURVE_TOP;
import static taco.agent.model.worldmodel.street.SegmentType.T_CROSSING;
import static taco.agent.model.worldmodel.street.SegmentType.X_CROSSING;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.SubLine;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.VectorUtils;

public class SegmentUtils
{
	private static final Map<SegmentType, Vector2D[][]> LINES;

	static
	{
		Map<SegmentType, Vector2D[][]> lines = new HashMap<>();
		lines.put(STRAIGHT, //
				new Vector2D[][] {new Vector2D[] {new Vector2D(0, 0.035), new Vector2D(1, 0.035)},
						new Vector2D[] {new Vector2D(0, 0.5), new Vector2D(1, 0.5)},
						new Vector2D[] {new Vector2D(0, 0.965), new Vector2D(1, 0.965)}});

		lines.put(STRAIGHT_WITH_CROSSWALK, lines.get(STRAIGHT));

		lines.put(CURVE_SMALL, //
				new Vector2D[][] {
						getQuarterCirclePoints(1.965, 7, 2), getQuarterCirclePoints(1.5, 7, 2),
						getQuarterCirclePoints(1.035, 7, 2),
				});

		lines.put(CURVE_BIG, //
				new Vector2D[][] {
						getQuarterCirclePoints(2.965, 7, 3), getQuarterCirclePoints(2.5, 7, 3),
						getQuarterCirclePoints(2.035, 7, 3),
				});

		lines.put(T_CROSSING, //
				new Vector2D[][] {
						new Vector2D[] {new Vector2D(0, 0.965), new Vector2D(1, 0.965)},
						new Vector2D[] {new Vector2D(0, 0.5), new Vector2D(1, 0.5)},
						new Vector2D[] {new Vector2D(0, 0.035), new Vector2D(0.035, 0.035), new Vector2D(0.035, 0)},
						new Vector2D[] {new Vector2D(1, 0.035), new Vector2D(0.965, 0.035), new Vector2D(0.965, 0)},
				});

		lines.put(X_CROSSING,	 //
				new Vector2D[][] {//
						new Vector2D[] {new Vector2D(0, 0.035), new Vector2D(0.035, 0.035), new Vector2D(0.035, 0)},
						new Vector2D[] {new Vector2D(1, 0.035), new Vector2D(0.965, 0.035), new Vector2D(0.965, 0)},
						new Vector2D[] {new Vector2D(0.035, 1), new Vector2D(0.035, 0.965), new Vector2D(0, 0.965)},
						new Vector2D[] {new Vector2D(0.965, 1), new Vector2D(0.965, 0.965), new Vector2D(1, 0.965)}});

		lines.put(PARKING_SPACE_VERTICAL, //
				new Vector2D[][] {		  //
						new Vector2D[] {
								new Vector2D(0, 0), new Vector2D(0, 0.8), new Vector2D(2, 0.8), new Vector2D(2, 0)},
						new Vector2D[] {new Vector2D(0.5, 0.8), new Vector2D(0.5, 0)},
						new Vector2D[] {new Vector2D(1, 0.8), new Vector2D(1, 0)},
						new Vector2D[] {new Vector2D(1.5, 0.8), new Vector2D(1.5, 0)}});

		lines.put(PARKING_SPACE_HORIZONTAL, //
				new Vector2D[][] {new Vector2D[] {new Vector2D(0, 0), new Vector2D(4, 0), new Vector2D(4, 0.5),
										  new Vector2D(0, 0.5), new Vector2D(0, 0)},
						new Vector2D[] {new Vector2D(0.8, 0.5), new Vector2D(0.8, 0)},
						new Vector2D[] {new Vector2D(1.6, 0.5), new Vector2D(1.6, 0)},
						new Vector2D[] {new Vector2D(2.4, 0.5), new Vector2D(2.4, 0)},
						new Vector2D[] {new Vector2D(3.2, 0.5), new Vector2D(3.2, 0)}});

		double delta = 0.13;
		lines.put(S_CURVE_TOP, //
				new Vector2D[][] {
						concat(true, getCirclePoints(1.965 + delta, 6, 0, 1 - delta, Angle.deg(21.3), Angle.deg(90)),
								getCirclePoints(1.035 + delta, 6, 3, 2 + delta, Angle.deg(-158.7), Angle.deg(-90))),
						//
						concat(true, getCirclePoints(1.5 + delta, 6, 0, 1 - delta, Angle.deg(21.3), Angle.deg(90)),
								getCirclePoints(1.5 + delta, 6, 3, 2 + delta, Angle.deg(-158.7), Angle.deg(-90))),
						//
						concat(true, getCirclePoints(1.035 + delta, 6, 0, 1 - delta, Angle.deg(21.3), Angle.deg(90)),
								getCirclePoints(1.965 + delta, 6, 3, 2 + delta, Angle.deg(-158.7), Angle.deg(-90))),
				});

		lines.put(S_CURVE_BOTTOM, //
				new Vector2D[][] {
						concat(false, getCirclePoints(1.965 + delta, 6, 0, 2 + delta, Angle.deg(-90), Angle.deg(-21.3)),
								getCirclePoints(1.035 + delta, 6, 3, 1 - delta, Angle.deg(90), Angle.deg(158.7))),
						//
						concat(false, getCirclePoints(1.5 + delta, 6, 0, 2 + delta, Angle.deg(-90), Angle.deg(-21.3)),
								getCirclePoints(1.5 + delta, 6, 3, 1 - delta, Angle.deg(90), Angle.deg(158.7))),
						//
						concat(false, getCirclePoints(1.035 + delta, 6, 0, 2 + delta, Angle.deg(-90), Angle.deg(-21.3)),
								getCirclePoints(1.965 + delta, 6, 3, 1 - delta, Angle.deg(90), Angle.deg(158.7)))});

		LINES = Collections.unmodifiableMap(lines);
	}

	public static List<StreetLine> getLines(Segment segment)
	{
		return transformLines(
				segment.getPosition(), segment.getRotation(), segment.getType(), LINES.get(segment.getType()));
	}

	private static Vector2D[] concat(boolean reverseFirst, Vector2D[] points1, Vector2D[] points2)
	{
		Vector2D[] result = new Vector2D[points1.length + points2.length - 1];
		if (reverseFirst) {
			Collections.reverse(Arrays.asList(points1));
		} else {
			Collections.reverse(Arrays.asList(points2));
		}
		System.arraycopy(points1, 0, result, 0, points1.length);
		System.arraycopy(points2, 1, result, points1.length, points2.length - 1);
		return result;
	}

	private static List<StreetLine> transformLines(Vector2D position, Angle angle, SegmentType type, Vector2D[][] lines)
	{
		List<StreetLine> transformed = new ArrayList<>();
		int count = 0;
		for (Vector2D[] line : lines) {
			Vector2D prev = null;
			for (Vector2D point : line) {
				if (prev != null) {
					Vector2D start = prev.add(position);
					Vector2D end = point.add(position);

					Vector2D pivot = getRotationPoint(type.width, type.height, angle).add(position);
					start = VectorUtils.rotateAround(start, pivot, angle);
					end = VectorUtils.rotateAround(end, pivot, angle);

					LineType lineType = LineType.OUTER;
					if (type.isParkingSpace()) {
						lineType = LineType.PARKING;
					} else if (count == 1 && type != X_CROSSING) {
						// we have made sure that the middle line is always defined second line
						lineType = LineType.MIDDLE;
					}
					transformed.add(new StreetLine(new SubLine(start, end, 0.0001), lineType));
				}

				prev = point;
			}
			count++;
		}

		return transformed;
	}

	public static Vector2D getRotationPoint(int width, int height, Angle angle)
	{
		double x = width / 2.0;
		double y = height / 2.0;

		// rectangular segments have to be rotated differently to pertain the location
		Direction direction = Direction.getDirection(angle);
		if (direction == Direction.WEST) {
			if (width > height) {
				x = x / width;
			} else if (width < height) {
				y = y / height;
			}
		}
		if (direction == Direction.EAST) {
			if (width > height) {
				y = x;
			} else if (width < height) {
				x = y;
			}
		}
		return new Vector2D(x, y);
	}

	private static Vector2D[] getQuarterCirclePoints(double radius, int segments, double centerX)
	{
		return getCirclePoints(radius, segments, centerX, 0, Angle.ANGLE_90, Angle.ANGLE_180);
	}

	/**
	 * @param radius the radius of the circle in m
	 * @param segments the number of lines with which to approximate a circle
	 * @param centerX the shift of x circle center coordinate
	 * @return an array of segments+1 points on the part of the circle specified
	 */
	private static Vector2D[] getCirclePoints(
			double radius, int segments, double centerX, double centerY, Angle angleStart, Angle angleEnd)
	{
		Vector2D[] result = new Vector2D[segments + 1];
		double angle = angleStart.radians();
		double howMuch = Math.abs(angleEnd.subtract(angleStart).degrees()) / 360;
		double angleInc = howMuch * 2 * Math.PI / segments;
		for (int i = 0; i < result.length; angle += angleInc, i++) {
			double x = radius * Math.cos(angle) + centerX;
			double y = radius * Math.sin(angle) + centerY;
			result[i] = new Vector2D(x, y);
		}
		return result;
	}

	public static Vector3D[] getLinePositions(IPose2D carPose, SubLine measurementLine, StreetMap map)
	{
		Vector2D start = carPose.applyTo(measurementLine.getSegments().get(0).getStart());
		Vector2D end = carPose.applyTo(measurementLine.getSegments().get(0).getEnd());
		Vector2D mid = start.add(end.subtract(start).scalarMultiply(0.5));

		Vector3D[] result = new Vector3D[3];
		SubLine detectionLine = new SubLine(start, end, 0.0001);
		List<Intersection> intersections = getLineIntersections(map, detectionLine);
		if (intersections.isEmpty()) {
			return result;
		}

		int mostMiddleIndex = SegmentUtils.getMidlineIndex(intersections, mid);

		if (mostMiddleIndex >= 0) {
			// we have a middle line, so take right and left with respect to middle
			result[1] = carPose.applyInverseTo(VectorUtils.to3D(intersections.get(mostMiddleIndex).getPoint()));
			if (mostMiddleIndex > 0) {
				result[0] = carPose.applyInverseTo(VectorUtils.to3D(intersections.get(mostMiddleIndex - 1).getPoint()));
			}
			if (mostMiddleIndex < intersections.size() - 1) {
				result[2] = carPose.applyInverseTo(VectorUtils.to3D(intersections.get(mostMiddleIndex + 1).getPoint()));
			}
		}
		return result;
	}

	private static List<Intersection> getLineIntersections(StreetMap map, SubLine line)
	{
		Vector2D start = line.getSegments().get(0).getStart();

		List<Intersection> intersections = new ArrayList<>();
		for (Segment segment : map) {
			if (segment.isParkingSpace()) {
				// for now we ignore parking spaces
				continue;
			}
			for (SegmentUtils.StreetLine streetLine : SegmentUtils.getLines(segment)) {
				Vector2D intersection = line.intersection(streetLine.line, true);
				if (intersection == null) {
					continue;
				}

				intersections.add(new Intersection(intersection, streetLine.type, start.distance(intersection)));
			}
		}

		if (intersections.isEmpty()) {
			return intersections;
		}

		Collections.sort(intersections);
		Iterator<Intersection> iterator = intersections.iterator();
		Intersection previous = null;
		// remove duplicates from lines being piecewise segments
		while (iterator.hasNext()) {
			Intersection current = iterator.next();
			if (previous != null && current.point.distance(previous.point) < 0.01) {
				iterator.remove();
			} else {
				previous = current;
			}
		}
		return intersections;
	}

	private static int getMidlineIndex(List<Intersection> intersections, Vector2D mid)
	{
		Iterator<Intersection> iterator = intersections.iterator();
		Intersection mostMiddle = null;
		int mostMiddleIndex = -1;
		int i = 0;
		while (iterator.hasNext()) {
			Intersection current = iterator.next();
			if (current.getType() == LineType.MIDDLE) {
				if (mostMiddle == null || mostMiddle.distance(mid) > current.distance(mid)) {
					mostMiddle = current;
					mostMiddleIndex = i;
				}
			}
			i++;
		}
		return mostMiddleIndex;
	}

	public enum LineType { MIDDLE, OUTER, PARKING, STOP }

	public static class StreetLine
	{
		public final SubLine line;

		public final LineType type;

		public StreetLine(SubLine line, LineType type)
		{
			this.line = line;
			this.type = type;
		}
	}
}
