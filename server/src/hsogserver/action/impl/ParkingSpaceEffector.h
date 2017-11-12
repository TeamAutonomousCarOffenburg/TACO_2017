#pragma once

#include "Effector.h"
#include "action/IParkingSpaceEffector.h"

namespace taco
{
class ParkingSpaceEffector : public Effector, public virtual IParkingSpaceEffector
{
  public:
	typedef boost::shared_ptr<ParkingSpaceEffector> Ptr;
	typedef boost::shared_ptr<const ParkingSpaceEffector> ConstPtr;

	ParkingSpaceEffector(const std::string &name) : Effector(name){};
	virtual ~ParkingSpaceEffector(){};

	virtual void setId(const int &id)
	{
		_id = id;
	}

	virtual void setPosX(const double &posX)
	{
		_posX = posX;
	}

	virtual void setPosY(const double &posY)
	{
		_posY = posY;
	}

	virtual void setState(const int &state)
	{
		_state = state;
	}

	virtual int getId() const
	{
		return _id;
	}

	virtual double getPosX() const
	{
		return _posX;
	}

	virtual double getPosY() const
	{
		return _posY;
	}

	virtual int getState() const
	{
		return _state;
	}

  protected:
	int _id = 0;
	double _posX = 0;
	double _posY = 0;
	int _state = 0;
};
}