#pragma once

#include <mutex>
#include <thread>

#include "../IObjectDetection.h"
#include "../IVisibleObject.h"
#include "utils/concurrency/BlockingQueue.h"

#include "IResource.h"
#include "PointCloudWorker.h"
#include "SensorFusion.h"
#include "meta/ICarMetaModel.h"

#include <pcl/filters/extract_indices.h>

namespace taco
{
/**
 * class manages obstacle detection
 *
 * \author Peter Walden
 */

class ObjectDetection : public virtual IObjectDetection, public virtual IResource
{
  public:
	ObjectDetection(ICarMetaModel::Ptr metamodel);
	virtual ~ObjectDetection();
	ObjectDetection(const ObjectDetection &);

	void start();
	void stop();
	void update();

	virtual void setData(IPointCloudPerceptor::ConstPtr pointCloudPerceptor,
			std::vector<IDoubleValuePerceptor::ConstPtr> perceptorsIR,
			std::vector<IDoubleValuePerceptor::ConstPtr> perceptorsUS);

	std::vector<IVisibleObject::Ptr> getObjectResult();

	virtual void setResult(std::vector<IVisibleObject::Ptr> results);
	virtual const IPointCloudPerceptor::ConstPtr getPointCloudPerceptor();
	virtual const ICameraConfig::ConstPtr getPointCloudConfig(std::string name);
	virtual const std::vector<AlignedBoundingBox3D::Ptr> getDistanceSensorData();
	virtual const std::vector<AlignedBoundingBox3D::Ptr> getPointCloudSensorData();
	virtual void setPointCloudSensorData(const std::vector<AlignedBoundingBox3D::Ptr> data);

  private:
	void processDistanceSensorData();
	void save(IDoubleValuePerceptor::ConstPtr perceptor);

	long _irLastMeasurementTime;
	long _usLastMeasurementTime;

	std::vector<IDoubleValuePerceptor::ConstPtr> _sensorsIR;
	std::vector<IDoubleValuePerceptor::ConstPtr> _sensorsUS;

	IPointCloudPerceptor::ConstPtr _pointCloudSensor;

	PointCloudWorker *_pcWorker;
	SensorFusion *_sensorFusion;
	std::map<const std::string, ISensorConfig::ConstPtr> _configmap;

	std::mutex _resultLock;
	std::mutex _dataLock;
	std::mutex _pointCloudDataLock;
	std::vector<IVisibleObject::Ptr> _results;
	BlockingQueue<pcl::PointXYZ, 500> *_distanceValues;
	std::vector<AlignedBoundingBox3D::Ptr> _pointCloudData;
};
}
