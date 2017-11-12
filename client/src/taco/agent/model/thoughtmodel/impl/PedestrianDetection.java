package taco.agent.model.thoughtmodel.impl;

import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Polygon;
import hso.autonomy.util.geometry.VectorUtils;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

public class PedestrianDetection extends Detection
{
	private double distance;

	public PedestrianDetection()
	{
		// we want 3 consecutive false observations before we believe
		super(1, 3);
		distance = 0.6;
	}

	@Override
	public void update(IAudiCupThoughtModel thoughtModel)
	{
		IAudiCupWorldModel worldModel = thoughtModel.getWorldModel();
		double validityTime = 0;
		if (isValid()) {
			if (distance < 0.6) {
				distance -= 0.01;
			} else {
				validityTime = getValidityTime(worldModel.getGlobalTime());
				distance = 0.6 - Geometry.getLinearFuzzyValue(5, 10, true, validityTime) * 0.6;
			}
		} else {
			if (getInValidityTime(worldModel.getGlobalTime()) > 3) {
				distance = 0.6;
			}
		}
		boolean result =
				worldModel.getRecognizedObjects()
						.stream()
						.filter(object -> object.getType().isPedestrian())
						.anyMatch(pedestrian -> {
							Polygon polygon = new Polygon(pedestrian.getArea());
							IPose2D carPose = worldModel.getThisCar().getPose();
							return thoughtModel.getDriveWay().isPositionInWay(
									VectorUtils.to2D(carPose.applyInverseTo(VectorUtils.to3D(polygon.getCentroid()))),
									distance);
						});

		setValidity(result, worldModel.getGlobalTime());
	}
}
