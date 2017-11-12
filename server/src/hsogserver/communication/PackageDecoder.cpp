#include "PackageDecoder.h"
#include "Handler/BoolEffectorHandler.h"
#include "Handler/DoubleEffectorHandler.h"
#include "Handler/ManeuverStatusEffectorHandler.h"
#include "Handler/PositionEffectorHandler.h"
#include "Handler/RoadSignEffectorHandler.h"
#include <AADCCar.h>
#include <communication/Handler/ObstacleEffectorHandler.h>
#include <communication/Handler/ParkingSpaceEffectorHandler.h>
#include <rapidjson/reader.h>

using namespace rapidjson;
using namespace std;

class aktuatorHandler
{
	taco::IAction::Ptr ap;
	std::string EffectorName;
	std::string currentName;
	SizeType EffectorNameLength;
	std::vector<IHandler *> Handlerlist;
	int level = 0;

	bool secondInt = false;
	bool Arrayfinished = false;
	bool ObjectStarted = false;

  public:
	aktuatorHandler(taco::IAction::Ptr ap, std::vector<IHandler *> Handlerlist)
	{
		this->ap = ap;
		this->Handlerlist = Handlerlist;
	}

	bool Null()
	{
		return false;
	}

	bool Bool(bool b)
	{
		return getHandler(EffectorName)->handle(currentName, &b);
	}

	bool Int(int i)
	{
		return getHandler(EffectorName)->handle(currentName, &i);
	}

	bool Uint(unsigned i)
	{
		int si = (int) i;
		return getHandler(EffectorName)->handle(currentName, &si);
	}

	bool Double(double d)
	{
		return getHandler(EffectorName)->handle(currentName, &d);
	}

	bool String(const char *str, SizeType length, bool copy)
	{
		return false;
	}

	bool StartObject()
	{
		if (!ObjectStarted) {
			ObjectStarted = true;
			return true;
		} else {
			level++;
			return true;
		}
	}

	bool EndObject(SizeType memberCount)
	{
		if (level == 0) {
			ObjectStarted = false;
			return true;
		} else {
			level--;
			return true;
		}
	}

	bool StartArray()
	{
		if (taco::AADCCar::MANEUVER_LIST.compare(EffectorName) == 0) {
			return true;
		}
		return false;
	}

	bool EndArray(SizeType memberCount)
	{
		return true;
	}

	bool Key(const char *str, SizeType length, bool copy)
	{
		currentName = str;
		if (level == 0) {
			EffectorName = str;
		}
		return true;
	}

	bool Int64(int64_t i)
	{
		return false;
	}

	bool Uint64(uint64_t i)
	{
		return false;
	}

	bool RawNumber(const char *str, SizeType length, bool copy)
	{
		return false;
	}

  private:
	IHandler *getHandler(std::string name)
	{
		for (std::vector<IHandler *>::iterator it = Handlerlist.begin(); it != Handlerlist.end(); it++) {
			if (it.operator*()->getName().compare(name) == 0) {
				return it.operator*();
			}
		}
		std::cout << "Error: No Handler for " << name << " found" << std::endl;
		return NULL;
	}
};

PackageDecoder::PackageDecoder(taco::IAction::Ptr ap, taco::ICarMetaModel::Ptr cm)
{
	this->ap = ap;
	auto lc = cm->getLightConfigs();
	for (unsigned int i = 0; i < lc.size(); i++) {
		Handlerlist.push_back(new BoolEffectorHandler(ap, lc[i]->getEffectorName()));
	}

	auto mc = cm->getMotorConfigs();
	for (unsigned int i = 0; i < mc.size(); i++) {
		Handlerlist.push_back(new DoubleEffectorHandler(ap, mc[i]->getEffectorName()));
	}

	auto sdc = cm->getServoDriveConfigs();
	for (unsigned int i = 0; i < sdc.size(); i++) {
		Handlerlist.push_back(new DoubleEffectorHandler(ap, sdc[i]->getEffectorName()));
	}

	auto msc = cm->getManeuverStatusConfigs();
	for (unsigned int i = 0; i < msc.size(); i++) {
		Handlerlist.push_back(new ManeuverStatusEffectorHandler(ap, msc[i]->getEffectorName()));
	}

	Handlerlist.push_back(new PositionEffectorHandler(ap, "Position"));
	Handlerlist.push_back(new RoadSignEffectorHandler(ap, "RoadSign"));
	Handlerlist.push_back(new ParkingSpaceEffectorHandler(ap, "ParkingSpace"));
	Handlerlist.push_back(new ObstacleEffectorHandler(ap, "Obstacle"));
}

PackageDecoder::~PackageDecoder()
{
	for (std::vector<IHandler *>::iterator it = Handlerlist.begin(); it != Handlerlist.end(); it++) {
		delete it.operator*();
	}
}

void PackageDecoder::decode(char *msg)
{
	Reader r;
	aktuatorHandler a(ap, Handlerlist);
	StringStream s(msg);
	r.Parse(s, a);
	ParseErrorCode errorCode = r.GetParseErrorCode();
	// kParseErrorDocumentRootNotSingular is ignored because of #26
	if (errorCode != ParseErrorCode::kParseErrorNone &&
			errorCode != ParseErrorCode::kParseErrorDocumentRootNotSingular) {
		std::cout << "Error Parsing JSON-String into Effectors" << std::endl;
		std::cout << errorCode << std::endl;
		std::cout << msg << std::endl;
	}
}
