/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizer;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Base class for Localizer components
 * @author Stefan Glaser
 */
public abstract class LocalizerBase implements ILocalizer
{
	@Override
	public boolean assignReferenceLines(IEnvironmentModel environment, Pose3D localizerInfo,
			List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		boolean assignedLine = false;
		final double range = 0.2;
		for (ILocalizationLine ll : lines) {
			for (IReferenceLine rl : environment.getReferenceLines()) {
				Vector3D position = localizerInfo.getPosition();
				Rotation orientation = localizerInfo.getOrientation();
				Vector3D llPos1 = position.add(orientation.applyTo(ll.getLocalPosition1()));
				Vector3D llPos2 = position.add(orientation.applyTo(ll.getLocalPosition2()));
				Vector3D rlPos1 = rl.getKnownPosition1();
				Vector3D rlPos2 = rl.getKnownPosition2();
				if (FuzzyCompare.eq(llPos1, rlPos1, range) && FuzzyCompare.eq(llPos2, rlPos2, range)) {
					assignReferenceLine(rl, ll, false);
					assignedLine = true;
				}
				if (FuzzyCompare.eq(llPos1, rlPos2, range) && FuzzyCompare.eq(llPos2, rlPos1, range)) {
					assignReferenceLine(rl, ll, true);
					assignedLine = true;
				}
			}
		}
		return assignedLine;

		// 1. idea
		// Create an utility system, in which we assign an utility (or likelyhood)
		// to each visible line and each reference line. After that we check, if
		// all reference lines are just preferred once. If this is the case, we
		// have a collision free assignment and hope everything is fine. If we
		// have collisions, we have to take further checks on the
		// second-most-preferred reference lines, etc.

		// 2. idea
		// Try to build a set of relations between all known reference lines
		// (like: "x.dir == y.dir && x.p1 == y.p1-(0,14) && x.p2 == y.p2-(0,14)"
		// or "x.p1 == refPoint.p", etc.) and then try to match the set of visible
		// lines in such a way, that no relation is violated.
		// Example:
		// 1. Normalize all vision information according to the normal of the
		// least square fitting plane through all line points (--> remove x- and
		// y-rotation).
		// 2. Take the longest unassigned line you see and try to find a unique
		// assignment with respect to all the other visible lines
		// 3. If an unique assignment was possible or not, remove the current line
		// from the set of lines to assign. Continue with Step 2 as long as the
		// set of lines to assign is not empty.
		// 4. If one run finished without assigning all lines, take the remaining
		// unassigned lines and restart with step 2. Stop if no single line could
		// be assigned during one run.

		// 3. idea
		// Try to start with an known reference point and search for lines which
		// end at the reference point and then start to assign lines from that
		// reference point on. Since all lines are interconnected with some other
		// lines, we can follow up the graph, until we don't see no matching
		// lines any more.
	}

	/**
	 * Assigns the given locLine to the refLine.<br>
	 * If <tt>crossAssignemnt == true</tt> the two points of the lines are
	 * assigned reverse (refLine.p1 = locLine.p1 && refLine.p2 = locLine.p2).<br>
	 *
	 * @param refLine - the reference line to update
	 * @param locLine - the unlabeled line, which should be assigned to the
	 *        refLine
	 * @param crossAssignment - cross assignment of points<br>
	 *        (<tt>false</tt> => refLine.p1 = locLine.p1 && refLine.p2 =
	 *        locLine.p2;<br>
	 *        <tt>true</tt> => refLine.p1 = locLine.p2 && refLine.p2 =
	 *        locLine.p1)
	 */
	protected void assignReferenceLine(IReferenceLine refLine, ILocalizationLine locLine, boolean crossAssignment)
	{
		Vector3D refLineKnownPos1 = refLine.getKnownPosition1();
		Vector3D refLineKnownPos2 = refLine.getKnownPosition2();
		Vector3D locLinePos1 = locLine.getLocalPosition1();
		Vector3D locLinePos2 = locLine.getLocalPosition2();

		if (crossAssignment) {
			refLine.updateLocalPositions(locLinePos2, locLinePos1);
		} else {
			refLine.updateLocalPositions(locLinePos1, locLinePos2);
		}

		refLine.adjustKnownPosition1(refLineKnownPos1);
		refLine.adjustKnownPosition2(refLineKnownPos2);
	}

	/**
	 * Extracts the visible reference points of the environment into a new list.
	 * If no reference point is visible, an empty list is returned.
	 *
	 * @param environment - the environment containing the reference points
	 * @return a new list containing just the visible reference points of the
	 *         environment
	 */
	protected List<IReferencePoint> getVisibleReferencePoints(IEnvironmentModel environment)
	{
		List<IReferencePoint> refPoints = new ArrayList<>();
		if (environment.getReferencePoints() != null) {
			for (IReferencePoint refPoint : environment.getReferencePoints()) {
				if (refPoint.isVisible()) {
					refPoints.add(refPoint);
				}
			}
		}

		return refPoints;
	}

	/**
	 * Checks if the environment contains any visible reference points.
	 *
	 * @param environment - the environment containing the reference points
	 * @return <tt>true</tt> if the environment contains any visible reference
	 *         point, else <tt>false</tt>
	 */
	protected boolean containsVisibleReferencePoints(IEnvironmentModel environment)
	{
		if (environment.getReferencePoints() != null) {
			for (IReferencePoint refPoint : environment.getReferencePoints()) {
				if (refPoint.isVisible()) {
					return true;
				}
			}
		}

		return false;
	}

	protected int numberOfVisibleReferencePoints(IEnvironmentModel environment)
	{
		int count = 0;
		if (environment.getReferencePoints() != null) {
			for (IReferencePoint refPoint : environment.getReferencePoints()) {
				if (refPoint.isVisible()) {
					count++;
				}
			}
		}

		return count;
	}

	/**
	 * Extracts the visible reference lines of the environment into a new list.
	 * If no reference line is visible, an empty list is returned.
	 *
	 * @param environment - the environment containing the reference lines
	 * @return a new list containing just the visible reference lines of the
	 *         environment
	 */
	protected List<IReferenceLine> getVisibleReferenceLines(IEnvironmentModel environment)
	{
		List<IReferenceLine> refLines = new ArrayList<>();

		if (environment.getReferenceLines() != null) {
			for (IReferenceLine refLine : environment.getReferenceLines()) {
				if (refLine.isVisible()) {
					refLines.add(refLine);
				}
			}
		}

		return refLines;
	}
}
