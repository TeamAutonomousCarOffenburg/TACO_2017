#include "SensorFusion.h"

#define CYCLES_PER_SECOND 10

typedef std::chrono::duration<int, std::ratio<1, CYCLES_PER_SECOND>> frame_duration;

using namespace taco;
using namespace Eigen;

SensorFusion::SensorFusion(IResource *resource) : _resource(resource)
{
}

void SensorFusion::run()
{
	while (!m_stop) {
		auto start_time = std::chrono::steady_clock::now();
		auto end_time = start_time + frame_duration(1);

		maintain(_recognizedBoxes);

		const std::vector<AlignedBoundingBox3D::Ptr> distanceData = _resource->getDistanceSensorData();
		const std::vector<AlignedBoundingBox3D::Ptr> cameraData = _resource->getPointCloudSensorData();

		// disable object detection for now because it's too unreliable

		// std::vector<IVisibleObject::Ptr> resultCamera = getObjectsFromCam(cameraData, _recognizedBoxes);
		auto resultCamera = std::vector<IVisibleObject::Ptr>();

		// std::vector<IVisibleObject::Ptr> resultDistance = getObjectsFromDistanceSensors(distanceData,
		// _recognizedBoxes);
		auto resultDistance = std::vector<IVisibleObject::Ptr>();

		std::vector<IVisibleObject::Ptr> result = resultCamera;

		// get rid of double objects
		std::vector<IVisibleObject::Ptr> independentObj;
		for (const IVisibleObject::Ptr &dObj : resultDistance) {
			bool match = false;
			for (const IVisibleObject::Ptr &cObj : resultCamera) {
				if (Geometry::isInsidePolygon(cObj->getBoundingPoly()->getPoints(), dObj->getPose().getPosition())) {
					match = true;
					break;
				}
			}
			if (!match) {
				independentObj.push_back(dObj);
			}
		}

		result.insert(result.end(), independentObj.begin(), independentObj.end());

		_resource->setResult(result);
		// Sleep if necessary
		std::this_thread::sleep_until(end_time);
	}
}

std::vector<IVisibleObject::Ptr> SensorFusion::getObjectsFromCam(
		const std::vector<AlignedBoundingBox3D::Ptr> &cameraData, std::vector<AlignedBoundingBox3D::Ptr> &knownBoxes)
{
	std::vector<IVisibleObject::Ptr> resultCamera = std::vector<IVisibleObject::Ptr>();
	if (!cameraData.empty()) {
		for (AlignedBoundingBox3D::Ptr box3d : cameraData) {
			bool match = false;
			// match with last seen boxes
			for (AlignedBoundingBox3D::Ptr &box : knownBoxes) {
				if (*box == *box3d) {
					++*box3d;
					box3d = boost::make_shared<AlignedBoundingBox3D>(*box + *box3d);
					// update knownBoxes reference in list
					box = box3d;
					match = true;
					break;
				}
			}
			if (!match) {
				// new box give max score.
				*box3d += CYCLES_PER_SECOND;
				knownBoxes.push_back(box3d);
			}

			AlignedBoundingBox3D::Ptr globalBox = boost::make_shared<AlignedBoundingBox3D>(*box3d);
			Polygon::Ptr polygon = map(globalBox);

			double xDiff = globalBox->maxPoint()(0) - globalBox->minPoint()(0);
			double yDiff = globalBox->maxPoint()(1) - globalBox->minPoint()(1);
			double zDiff = globalBox->maxPoint()(2) - globalBox->minPoint()(2);

			IVisibleObject::Ptr object;
			// check if road sign
			if ((((xDiff > 0.008 && xDiff < 0.036) && (yDiff > 0.12 && yDiff < 0.16)) ||
						((yDiff > 0.008 && yDiff < 0.036) && (xDiff > 0.12 && xDiff < 0.16))) &&
					(zDiff > 0.23 && zDiff < 0.25)) {
				object = boost::make_shared<RoadSign>(
						RoadSign(SignType::Unknown, Pose2D(box3d->origin()(0), box3d->origin()(1))));
			} else {
				object = boost::make_shared<VisibleObject>(
						VisibleObject("Object", Pose2D(box3d->origin()(0), box3d->origin()(1)), polygon));
			}
			resultCamera.push_back(object);
		}
	}
	return resultCamera;
}

std::vector<IVisibleObject::Ptr> SensorFusion::getObjectsFromDistanceSensors(
		const std::vector<AlignedBoundingBox3D::Ptr> &distanceData, std::vector<AlignedBoundingBox3D::Ptr> &knownBoxes)
{
	std::vector<IVisibleObject::Ptr> resultDistance = std::vector<IVisibleObject::Ptr>();
	if (!distanceData.empty()) {
		for (AlignedBoundingBox3D::Ptr box3d : distanceData) {
			bool match = false;
			// set score limit to one scond
			box3d->setScoreLimit(CYCLES_PER_SECOND);
			// match with last seen boxes
			for (AlignedBoundingBox3D::Ptr &box : knownBoxes) {
				if (*box == *box3d) {
					++*box3d;
					box3d = boost::make_shared<AlignedBoundingBox3D>(*box + *box3d);
					box = box3d;
				}
			}
			if (!match) {
				// new box give max score.
				*box3d += CYCLES_PER_SECOND;
				knownBoxes.push_back(box3d);
			}

			Polygon::Ptr polygon = map(box3d);

			// matching auf objekte
			IVisibleObject::Ptr object = IVisibleObject::Ptr(
					new VisibleObject("Object", Pose2D(box3d->origin()(0), box3d->origin()(1)), polygon));
			resultDistance.push_back(object);
		}
	}
	return resultDistance;
}

void SensorFusion::maintain(std::vector<AlignedBoundingBox3D::Ptr> &boxes)
{
	// do maintaince, reduce score of box
	std::vector<AlignedBoundingBox3D::Ptr> valid;
	for (auto knownBox : boxes) {
		// reduce score
		--*knownBox;
		// check score
		if (*knownBox > 0) {
			valid.push_back(knownBox);
		}
	}
	boxes = valid;
}

// polygon is local to AlignedBoundingBox3D origin
Polygon::Ptr SensorFusion::map(AlignedBoundingBox3D::Ptr &box3d)
{
	Vector2d min(box3d->minPoint()(0), box3d->minPoint()(1));
	Vector2d max(box3d->maxPoint()(0), box3d->maxPoint()(1));

	// remaining vertices
	Vector2d a = Vector2d(min(0), max(1));
	Vector2d b = Vector2d(max(0), min(1));

	Polygon poly(min, a, max, b);

	return boost::make_shared<Polygon>(poly);
}
