/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.localizer.impl;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizer;
import hso.autonomy.util.geometry.Pose3D;

/**
 * Composite localizer, calling a set of other localizer sequentially until a
 * localization / line assignment was successful.
 *
 * @author Stefan Glaser
 */
public class CompositeLocalizer extends LocalizerBase
{
	private ILocalizer[] localizers;

	public CompositeLocalizer(ILocalizer[] localizers)
	{
		this.localizers = localizers;
	}

	@Override
	public Pose3D localize(IEnvironmentModel environment, List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		if (!containsVisibleReferencePoints(environment)) {
			// there is no visible flag
			return null;
		}

		Pose3D result = null;

		for (ILocalizer localizer : localizers) {
			if (result == null) {
				try {
					result = localizer.localize(environment, lines, estimatedOrientation);
				} catch (Exception e) {
					// in case a localizer fails we want to be able to continue with
					// the others
				}
			}
		}

		return result;
	}

	@Override
	public boolean assignReferenceLines(IEnvironmentModel environment, Pose3D localizerInfo,
			List<ILocalizationLine> lines, Rotation estimatedOrientation)
	{
		if (numberOfVisibleReferencePoints(environment) < 2) {
			// not enough visible reference points
			return false;
		}

		boolean result = false;

		for (ILocalizer localizer : localizers) {
			if (!result) {
				result = localizer.assignReferenceLines(environment, localizerInfo, lines, estimatedOrientation);
			}
		}

		return result;
	}
}
