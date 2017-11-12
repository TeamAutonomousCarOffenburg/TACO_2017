package taco.agent.decision.behavior.impl;

import java.awt.Color;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.SubLine;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.VectorUtils;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.worldmodel.street.SegmentUtils;
import taco.agent.model.worldmodel.street.StreetMap;

public class FollowRightLaneLearning extends FollowRightLane
{
	private boolean finished;

	private boolean starting;

	private boolean closeToStart;

	private IPose2D startPose;

	private SubLine measurementLine;

	private double avgMidlineDistance;

	private int avgCount;

	/** the values to check. This is only necessary to specify in this visualization. The real learning does not need
	 * this! */
	private float[] valuesToCheck = {0.07f, 0.06f, 0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f};

	private int valuesToCheckCounter;

	private float bestInputFactor;

	private float bestInputFactorUtility;

	private int pauseCounter;

	private int drawCounter;

	public FollowRightLaneLearning(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.FOLLOW_RIGHT_LANE_LEARNING, thoughtModel, null);
		// we start with a sub optimal input Factor
		finished = false;
		starting = true;
		closeToStart = true;
		measurementLine = new SubLine(new Vector2D(0.18, -1), new Vector2D(0.18, 1), 0.0001);
		avgMidlineDistance = 0;
		avgCount = 0;
		valuesToCheckCounter = 0;
		inputFactor = valuesToCheck[valuesToCheckCounter];
		bestInputFactor = inputFactor;
		bestInputFactorUtility = 100;
		pauseCounter = 0;
		drawCounter = 0;
	}

	@Override
	public void perform()
	{
		if (finished || valuesToCheckCounter >= valuesToCheck.length - 1) {
			// finished
			finished = true;
			getAgentModel().getLight(LightName.INDICATOR_LEFT).turnOff();
			getAgentModel().getLight(LightName.INDICATOR_RIGHT).turnOff();
			getAgentModel().getLight(LightName.WARN).turnOn();
			getAgentModel().getMotor().stop();
			return;
		}

		if (pauseCounter > 0) {
			// make a pause
			getAgentModel().getMotor().stop();
			pauseCounter--;
			return;
		}

		setSpeed(35);
		IPose2D currentPose = getWorldModel().getThisCar().getPose();

		if (starting) {
			startPose = currentPose;
			starting = false;
		}

		double distanceToStart = currentPose.getPosition().distance(startPose.getPosition());
		if (distanceToStart < 1) {
			if (!closeToStart) {
				// we have made a round
				if (avgMidlineDistance < bestInputFactorUtility) {
					bestInputFactor = inputFactor;
					bestInputFactorUtility = (float) avgMidlineDistance;
				} else if (valuesToCheckCounter > 2) {
					finished = true;
				}
				avgMidlineDistance = 0;
				avgCount = 0;
				inputFactor = valuesToCheck[++valuesToCheckCounter];
				closeToStart = true;
				pauseCounter = 200;
				return;
			}
		} else {
			closeToStart = false;
		}

		// measure precision
		StreetMap map = getWorldModel().getMap();
		Vector3D middle = SegmentUtils.getLinePositions(currentPose, measurementLine, map)[1];
		double distance = 1.5;
		if (middle != null) {
			distance = Vector3D.distance(new Vector3D(0.18, 0.225, 0), middle);
			Vector2D globalStart = currentPose.applyTo(new Vector2D(0.18, 0.225));
			Vector2D globalEnd = currentPose.applyTo(VectorUtils.to2D(middle));
			getThoughtModel().getDrawings().draw(
					"followline" + drawCounter, Color.RED, new SubLine(globalStart, globalEnd, 0.0001));
		}
		avgMidlineDistance = (avgMidlineDistance * avgCount + distance) / (avgCount + 1);
		avgCount++;
		drawCounter = (drawCounter + 1) % 200;

		drawValues();

		super.perform();
	}

	protected void drawValues()
	{
		double x = -4.2;
		double y = 4.2;
		float scaling = 0.004f;
		getThoughtModel().getDrawings().draw(
				BEHAVIOR_DRAWING + 1, Color.WHITE, "Parameter       : " + inputFactor, new Vector2D(x, y), scaling);

		String text = String.format("%6.4f", avgMidlineDistance);
		Color color = Color.RED;
		if (avgMidlineDistance < bestInputFactorUtility) {
			color = Color.GREEN;
		}
		getThoughtModel().getDrawings().draw( //
				BEHAVIOR_DRAWING + 2, color, "Messung          : " + text, new Vector2D(x, y -= 0.3), scaling);

		getThoughtModel().getDrawings().draw(BEHAVIOR_DRAWING + 3, Color.WHITE, "Bester Wert     : " + bestInputFactor,
				new Vector2D(x, y -= 0.3), scaling);

		text = "-";
		color = Color.WHITE;
		if (bestInputFactorUtility < 99) {
			text = String.format("%6.4f", bestInputFactorUtility);
			color = Color.GREEN;
			if (avgMidlineDistance < bestInputFactorUtility) {
				color = Color.RED;
			}
		}
		getThoughtModel().getDrawings().draw(
				BEHAVIOR_DRAWING + 4, color, "Beste Messung: " + text, new Vector2D(x, y -= 0.3), scaling);
	}
}
