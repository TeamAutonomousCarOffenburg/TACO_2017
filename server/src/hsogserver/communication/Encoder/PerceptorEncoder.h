#pragma once

#include "../../perception/IPerception.h"
#include "IEncoder.h"

// Abstrakte Basisklasse für das Codieren eines Perceptors
template <class PERC> class PerceptorEncoder : public IEncoder
{
  public:
	void encode(Writer<StringBuffer> *writer)
	{
		typename PERC::ConstPtr perceptor = getPerceptor();
		if (perceptor) {
			writer->Key(PerceptorName.c_str());
			writer->StartObject();
			writer->Key("timestamp");
			writer->Int64(perceptor->getTime());
			writeValue(writer, perceptor);
			writer->EndObject();
		}
	}

	virtual ~PerceptorEncoder()
	{
	}

  protected:
	// Im Konstruktor wird ein Pointer auf das PerceptionObjekt und der PerceptorName übergeben.
	PerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName)
	{
		P = p;
		PerceptorName = perceptorName;
	}
	std::string PerceptorName;
	taco::IPerception::Ptr P;

	// Virtuelle Methode. Hier der/die Wert/e des Perceptors mithilfe des Writers geschrieben.
	// Wird von der encode-Methode Aufgerufen
	virtual void writeValue(Writer<StringBuffer> *writer, typename PERC::ConstPtr perceptor) = 0;

  private:
	typename PERC::ConstPtr getPerceptor()
	{
		return boost::dynamic_pointer_cast<const PERC>(P->getPerceptor(PerceptorName));
	}
};
