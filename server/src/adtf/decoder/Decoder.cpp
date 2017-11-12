#include "Decoder.h"

using namespace taco;

Decoder::Decoder(IEventLogger::ConstPtr logger)
{
	_logger = logger;
}

Decoder::~Decoder()
{
}

int Decoder::indexOfPin(const InputPin &pin) const
{
	for (unsigned int i = 0; i < _pins.size(); i++) {
		if (_pins[i] == pin) {
			return i;
		}
	}

	return -1;
}

const vector<InputPin> &Decoder::getPinConfig() const
{
	return _pins;
}

void Decoder::clearReceived()
{
	for (unsigned int i = 0; i < _pins.size(); i++) {
		_pins[i].received = false;
	}
}

bool Decoder::allPinsReceived()
{
	for (unsigned int i = 0; i < _pins.size(); i++) {
		if (!_pins[i].received) {
			return false;
		}
	}

	return true;
}
