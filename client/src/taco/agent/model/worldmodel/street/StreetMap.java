package taco.agent.model.worldmodel.street;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.IPose2D;

/**
 * Represents an offline map of streets
 */
public class StreetMap implements Iterable<Segment>
{
	/** the segment we keep as root point of the graph */
	private Segment rootSegment;

	/** The list of drive sectors. */
	private List<Sector> sectors;

	public StreetMap(Segment rootSegment)
	{
		this.rootSegment = rootSegment;
		this.sectors = new ArrayList<>();
	}

	public Segment getRootSegment()
	{
		return rootSegment;
	}

	/**
	 * @param id unique identifier of the segment to find
	 * @return the segment with specified id, null if no such exists
	 */
	public Segment getSegment(int id)
	{
		return getSegments().stream().filter(segment -> segment.id == id).findFirst().orElse(null);
	}

	@Override
	public Iterator<Segment> iterator()
	{
		return getSegments().iterator();
	}

	/**
	 * @param position the position to check
	 * @return the segment containing the passed position, null if none does
	 */
	public Segment getSegmentContaining(Vector3D position)
	{
		return getSegments().stream().filter(segment -> segment.contains(position)).findFirst().orElse(null);
	}

	/**
	 * Same as getSegmentContaining, but this one searches for the segment that fits only with X- or Y-coordinates, not
	 * complete position
	 * @param position the position to check
	 * @param checkedSegments list of already found elements as additional filter
	 * @return the segment containing the passed position, null if none does
	 */
	public Segment getSegmentContainingOnlyXorY(Vector3D position, List<Segment> checkedSegments)
	{
		return getSegments()
				.stream()
				.filter(segment -> !checkedSegments.contains(segment))
				.filter(segment
						-> (segment.getArea().containsX((float) position.getX()) &&
								   Math.abs(segment.getPosition().getY() - position.getY()) <= 1.2) ||
								   (segment.getArea().containsY((float) position.getY()) &&
										   Math.abs(segment.getPosition().getX() - position.getX()) <= 1.2))

				.findAny()
				.orElse(null);
	}

	/** returns a flat list of all segments in the map */
	private List<Segment> getSegments()
	{
		ArrayList<Segment> segments = new ArrayList<>();
		collectSegments(rootSegment, segments);
		return segments;
	}

	private void collectSegments(Segment currentSegment, List<Segment> segments)
	{
		if (currentSegment == null || segments.contains(currentSegment)) {
			return;
		}
		segments.add(currentSegment);

		if (currentSegment.hasAttachedOption()) {
			collectSegments(currentSegment.getAttachedOption().segmentAfter, segments);
		}

		for (SegmentLink outOption : currentSegment.outOptions) {
			if (outOption != null) {
				collectSegments(outOption.segmentAfter, segments);
			}
		}
	}

	public Area2D.Float getArea()
	{
		double minX = Double.POSITIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY;
		double maxX = Double.NEGATIVE_INFINITY;
		double maxY = Double.NEGATIVE_INFINITY;

		for (Segment segment : this) {
			double x = segment.getPosition().getX();
			double y = segment.getPosition().getY();
			Area2D.Float area = new Area2D.Float(x, x + segment.getType().width, y, y + segment.getType().height);
			if (area.getMinX() < minX) {
				minX = area.getMinX();
			}
			if (area.getMinY() < minY) {
				minY = area.getMinY();
			}
			if (area.getMaxX() > maxX) {
				maxX = area.getMaxX();
			}
			if (area.getMaxY() > maxY) {
				maxY = area.getMaxY();
			}
		}

		return new Area2D.Float(minX, maxX, minY, maxY);
	}

	public IPose2D getCurrentStartPose(int sectorIndex)
	{
		return sectors.get(sectorIndex).getStartPose();
	}

	public Segment getCurrentStartSegment(int sectorIndex)
	{
		return getSegment(sectors.get(sectorIndex).getSegmentIndex());
	}

	public List<Sector> getSectors()
	{
		return Collections.unmodifiableList(sectors);
	}

	public void addSector(Sector sector)
	{
		sectors.add(sector);
	}
}