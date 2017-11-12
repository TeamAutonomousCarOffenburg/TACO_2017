#include "ADTFPinMessageEncoder.h"
#include <AADCCar.h>
#include <action/IObstacleEffector.h>
#include <action/IParkingSpaceEffector.h>
#include <action/IRoadSignEffector.h>

using namespace taco;
using namespace std;

ADTFPinMessageEncoder::ADTFPinMessageEncoder(IAction::Ptr action, ICarMetaModel::ConstPtr carMetaModel)
	: _action(action)
{
	for (const auto &servoDriveConfig : carMetaModel->getServoDriveConfigs()) {
		_pins.emplace_back(servoDriveConfig->getEffectorName(), "tSignalValue");
	}

	for (const auto &config : carMetaModel->getMotorConfigs()) {
		_pins.emplace_back(config->getEffectorName(), "tSignalValue");
	}

	for (const auto &config : carMetaModel->getLightConfigs()) {
		_pins.emplace_back(config->getEffectorName(), "tBoolSignalValue");
	}

	for (const auto &config : carMetaModel->getManeuverStatusConfigs()) {
		_pins.emplace_back(config->getEffectorName(), "tDriverStruct");
	}

#if TACO_CONFIG == TACO_2017
	_pins.emplace_back("Position", "tPosition");
	_pins.emplace_back("RoadSign", "tTrafficSign");
	_pins.emplace_back("ParkingSpace", "tParkingSpace");
	_pins.emplace_back("Obstacle", "tObstacle");
#endif
}

ADTFPinMessageEncoder::~ADTFPinMessageEncoder() = default;

int ADTFPinMessageEncoder::indexOfPin(OutputPin pin) const
{
	for (unsigned int i = 0; i < _pins.size(); i++) {
		if (_pins[i] == pin) {
			return i;
		}
	}

	return -1;
}

const std::vector<OutputPin> &ADTFPinMessageEncoder::getOutputPins()
{
	return _pins;
}

bool ADTFPinMessageEncoder::encode(
		const OutputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);
	bool toTransmit = false;

	if (pinIndex >= 0) {
		cObjectPtr<adtf::IMediaSerializer> serializer;
		mediaTypeDescription->GetMediaSampleSerializer(&serializer);

		tInt size = serializer->GetDeserializedSize();
		mediaSample->AllocBuffer(size);

		cObjectPtr<adtf::IMediaCoder> mediaCoder;

		tUInt32 timestamp = 0;

		if (pin.signalType == "tBoolSignalValue") {
			IBoolValueEffector::Ptr boolEffector = _action->getLightEffector(pin.name);
			if (boolEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tBool value = boolEffector->getValue();
				mediaCoder->Set("bValue", (tVoid *) &value);
				mediaCoder->Set("ui32ArduinoTimestamp", (tVoid *) &timestamp);

				toTransmit = true;
			}
		} else if (pin.signalType == "tSignalValue") {
			IDoubleValueEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IDoubleValueEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tFloat32 value = valueEffector->getValue();
#if TACO_CONFIG == TACO_2017
				if (pin.name == AADCCar::MAIN_MOTOR) {
					value *= -0.4;
				} else if (pin.name == AADCCar::STEERING_SERVO) {
					value *= -1;
				}
#endif
				mediaCoder->Set("f32Value", (tVoid *) &value);
				mediaCoder->Set("ui32ArduinoTimestamp", (tVoid *) &timestamp);

				toTransmit = true;
			}
		} else if (pin.signalType == "tDriverStruct") {
			IManeuverStatusEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IManeuverStatusEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				int state = valueEffector->getStatus();
				int maneuverId = valueEffector->getManeuverId();
				mediaCoder->Set("i8StateID", (tVoid *) &state);
				mediaCoder->Set("i16ManeuverEntry", (tVoid *) &maneuverId);

				toTransmit = true;
			}
		} else if (pin.signalType == "tPosition") {
			IPositionEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IPositionEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tFloat32 f32x = static_cast<tFloat32>(valueEffector->getPosX());
				tFloat32 f32y = static_cast<tFloat32>(valueEffector->getPosY());
				tFloat32 f32radius = NAN;
				tFloat32 f32speed = NAN;
				tFloat32 f32heading = static_cast<tFloat32>(valueEffector->getAngle());
				mediaCoder->Set("f32x", (tVoid *) &f32x);
				mediaCoder->Set("f32y", (tVoid *) &f32y);
				mediaCoder->Set("f32radius", (tVoid *) &f32radius);
				mediaCoder->Set("f32speed", (tVoid *) &f32speed);
				mediaCoder->Set("f32heading", (tVoid *) &f32heading);

				toTransmit = true;
			}
		} else if (pin.signalType == "tTrafficSign") {
			IRoadSignEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IRoadSignEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tInt16 i16Identifier = static_cast<tInt16>(valueEffector->getId());
				tFloat32 f32x = static_cast<tFloat32>(valueEffector->getPosX());
				tFloat32 f32y = static_cast<tFloat32>(valueEffector->getPosY());
				tFloat32 f32angle = static_cast<tFloat32>(valueEffector->getAngle());
				mediaCoder->Set("i16Identifier", (tVoid *) &i16Identifier);
				mediaCoder->Set("f32x", (tVoid *) &f32x);
				mediaCoder->Set("f32y", (tVoid *) &f32y);
				mediaCoder->Set("f32angle", (tVoid *) &f32angle);

				toTransmit = true;
			}
		} else if (pin.signalType == "tParkingSpace") {
			IParkingSpaceEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IParkingSpaceEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tInt16 i16Identifier = static_cast<tInt16>(valueEffector->getId());
				tFloat32 f32x = static_cast<tFloat32>(valueEffector->getPosX());
				tFloat32 f32y = static_cast<tFloat32>(valueEffector->getPosY());
				tUInt16 ui16Status = static_cast<tUInt16>(valueEffector->getState());
				mediaCoder->Set("i16Identifier", (tVoid *) &i16Identifier);
				mediaCoder->Set("f32x", (tVoid *) &f32x);
				mediaCoder->Set("f32y", (tVoid *) &f32y);
				mediaCoder->Set("ui16Status", (tVoid *) &ui16Status);

				toTransmit = true;
			}
		} else if (pin.signalType == "tObstacle") {
			IObstacleEffector::Ptr valueEffector =
					boost::dynamic_pointer_cast<IObstacleEffector>(_action->getEffector(pin.name));
			if (valueEffector) {
				__adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
				if (!mediaCoder) {
					return false;
				}
				tFloat32 f32x = static_cast<tFloat32>(valueEffector->getPosX());
				tFloat32 f32y = static_cast<tFloat32>(valueEffector->getPosY());
				mediaCoder->Set("f32x", (tVoid *) &f32x);
				mediaCoder->Set("f32y", (tVoid *) &f32y);

				toTransmit = true;
			}
		}
	}

	return toTransmit;
}
