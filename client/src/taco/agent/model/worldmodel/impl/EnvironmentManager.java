package taco.agent.model.worldmodel.impl;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.communication.perception.IEnvironmentConfigPerceptor;
import taco.agent.model.agentmodel.IParkingSpaceActuator;
import taco.agent.model.agentmodel.IRoadSignActuator;
import taco.agent.model.worldmodel.ParkingSpaceState;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.agent.model.worldmodel.signdetection.RoadSignUtils;
import taco.agent.model.worldmodel.street.Direction;
import taco.agent.model.worldmodel.street.ParkingSegment;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.util.SignType;

import java.util.*;
import java.util.stream.Collectors;

public class EnvironmentManager
{
	private final IParkingSpaceActuator parkingSpaceActuator;

	private final IRoadSignActuator roadSignActuator;

	/** map with the parking spaces that are updated by our detection */
	private Map<Integer, ParkingSpace> parkingSpaces = new HashMap<>();

	/** map with the parking spaces from the roadsign.xml */
	private Map<Integer, ParkingSpace> parkingSpacesFromFile = new HashMap<>();

	/** list with the roadsigns that are updated by our detection */
	private List<RoadSign> knownRoadSigns = new ArrayList<>();

	/** list with the roadsigns from the roadsign.xml */
	private List<RoadSign> roadSignsFromFile = new ArrayList<>();

	private boolean initialized = false;

	public EnvironmentManager(
			IParkingSpaceActuator parkingSpaceActuator, IRoadSignActuator roadSignActuator, StreetMap map)
	{
		this.parkingSpaceActuator = parkingSpaceActuator;
		this.roadSignActuator = roadSignActuator;
		extractParkingSpaces(map);
		extractRoadSigns(map);
	}

	private void extractParkingSpaces(StreetMap map)
	{
		for (Segment segment : map) {
			if (!segment.isParkingSpace()) {
				continue;
			}

			for (int i = 0; i < Direction.values().length; i++) {
				Direction direction = Direction.values()[i];
				ParkingSegment parking = (ParkingSegment) segment;

				if (!parking.hasInOption(direction)) {
					continue;
				}

				int id = parking.getFirstID() + i;
				ParkingSpaceState state = ParkingSpaceState.FREE;
				if (parking.isOccupied(i)) {
					state = ParkingSpaceState.OCCUPIED;
				}
				IPose2D pose = parking.getOutOptionByRelativeParkingID(i).getPose();
				// for some historic reason the pose of a parking space seems to point to the left seen from
				// street. TODO pointing inward would be more natural.
				pose = new Pose2D(pose.getX(), pose.getY(), pose.getAngle().subtract(Angle.ANGLE_90));
				parkingSpaces.put(id, new ParkingSpace(id, pose, state));
			}
		}
	}

	private void extractRoadSigns(StreetMap map)
	{
		for (Segment segment : map) {
			for (Direction direction : Direction.values()) {
				if (segment.hasInOption(direction)) {
					knownRoadSigns.addAll(segment.getRoadSigns(direction));
				}
			}
		}
	}

	public void update(IEnvironmentConfigPerceptor perceptor, StreetMap map)
	{
		if (!initialized) {
			perceptor.getParkingSpaces().forEach(space -> parkingSpacesFromFile.put(space.getID(), space));

			roadSignsFromFile = perceptor.getRoadSigns();
			roadSignsFromFile.forEach(sign -> RoadSignUtils.loadReceivedRoadSignIntoMap(map, knownRoadSigns, sign, 0));

			initialized = true;
		}
	}

	public Map<Integer, ParkingSpace> getParkingSpaces()
	{
		return parkingSpaces;
	}

	public ParkingSpace getParkingSpaceById(int id)
	{
		return parkingSpaces.get(id);
	}

	public ParkingSpace getClosestParkingSpace(IPose2D source)
	{
		ParkingSpace closest = null;
		for (ParkingSpace space : parkingSpaces.values()) {
			if (closest == null || source.getDistanceTo(space.getPose()) < source.getDistanceTo(closest.getPose())) {
				closest = space;
			}
		}
		return closest;
	}

	public List<RoadSign> getKnownRoadSigns()
	{
		return knownRoadSigns;
	}

	public List<RoadSign> getVisibleRoadSigns()
	{
		return knownRoadSigns.stream().filter(RoadSign::isVisible).collect(Collectors.toList());
	}

	public void updateParkingSpace(int id, ParkingSpaceState state, float globalTime)
	{
		ParkingSpace parkingSpace = parkingSpaces.get(id);
		if (parkingSpace == null) {
			return;
		}

		parkingSpace.update(state, globalTime);

		// only set the parkingSpace to the actuator if the state has changed
		ParkingSpace parkingSpaceFromFile = parkingSpacesFromFile.get(id);
		if (parkingSpaceFromFile != null && parkingSpace.getState() != parkingSpaceFromFile.getState()) {
			parkingSpaceActuator.setParkingSpace(parkingSpace);
			parkingSpacesFromFile.put(id, parkingSpace);
		}
	}

	public void updateRoadSign(RoadSign newRoadSign)
	{
		RoadSign closestRoadSign = RoadSignUtils.findClosestRoadSign(knownRoadSigns, newRoadSign);
		if (closestRoadSign != null) {
			knownRoadSigns.remove(closestRoadSign);
		}
		knownRoadSigns.add(newRoadSign);
		// if the same sign was not found, update the actuator to send the new sign to backend
		roadSignActuator.setRoadSign(newRoadSign);
	}

	private void reportRemovedSign(RoadSign removedSign)
	{
		knownRoadSigns.remove(removedSign);
		roadSignActuator.setRoadSign(new RoadSign(SignType.SIGN_REMOVED, removedSign.getPose()));
	}

	public RoadSign updateVisisbleRoadSigns(IPose2D carPose, float globalTime)
	{
		RoadSign removedSign = null;
		for (RoadSign knownRoadSign : knownRoadSigns) {
			knownRoadSign.update(RoadSignUtils.isInVisibleArea(carPose, knownRoadSign), false, globalTime);
			if (knownRoadSign.hasBeenRemoved(globalTime)) {
				removedSign = knownRoadSign;
			}
		}

		if (removedSign != null) {
			reportRemovedSign(removedSign);
		}
		return removedSign;
	}
}
