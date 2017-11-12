#include "Communication.h"
#include "ByteConverter.h"
#include <AADCCar.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <utils/configurationreader/ConfigurationReader.h>

using namespace std;
using namespace std::chrono;
using namespace taco;

void Communication::start()
{
	serverRunning = true;
	receiveThread = thread(&Communication::receive, this);
}

void Communication::openSocket()
{
	socketFD = socket(AF_INET, SOCK_STREAM, 0);
	if (socketFD == -1) {
		error("socket()");
		return;
	}
	fcntl(socketFD, F_SETFL, O_NONBLOCK);

	int enable = 1;
	if (setsockopt(socketFD, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
		error("setsockopt(SO_REUSEADDR)");
	}

	struct sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(port);

	if (bind(socketFD, (struct sockaddr *) &address, sizeof(address)) == -1) {
		error("bind()");
		return;
	}

	if (listen(socketFD, 5) == -1) {
		error("listen()");
		return;
	}

	clientResponsive = true;
	currentTime = std::chrono::system_clock::now();

	log("Waiting for incoming connections");
}

void Communication::stop()
{
	serverRunning = false;
	close(clientFD);
	close(socketFD);
	receiveThread.join();
}

Communication::Communication(ICarMetaModel::Ptr carMetaModel, IAction::Ptr action, IPerception::Ptr perception,
		ILaneDetection::Ptr laneDetection, IObjectDetection::Ptr objectDetection, FloorNormalDetection &floorNormal,
		uint16_t port)
{
	EnvironmentConfiguration::Ptr environmentConfig = boost::make_shared<EnvironmentConfiguration>();
	this->encoder = new PackageEncoder(
			perception, carMetaModel, laneDetection, objectDetection, floorNormal, environmentConfig);
	this->decoder = new PackageDecoder(action, carMetaModel);
	this->port = port;
	socketFD = -1;
	clientFD = -1;
	serverRunning = false;
	_carMetaModel = carMetaModel;
	_action = action;
}

Communication::~Communication()
{
	delete encoder;
	delete decoder;
}

bool Communication::update()
{
	ssize_t n = 0;
	if (clientFD != -1) {
		unsigned char buffer[5000000];
		size_t length = encoder->encode(buffer, lastClientConnectionTime);
		if (length > 6) {
			n = write(clientFD, buffer, length);
		}
	}
	if (n == -1) {
		error("write()");
		return false;
	}
	return true;
}

void Communication::receive()
{
	while (serverRunning) {
		this_thread::sleep_for(std::chrono::milliseconds(5));

		if (socketFD == -1) {
			openSocket();
		}

		// wait until client is connected
		clientFD = accept(socketFD, nullptr, nullptr);
		if (clientFD == -1 && (errno != EAGAIN || errno != EWOULDBLOCK)) {
			error("accept()");
			return;
		}

		if (clientFD == -1) {
			continue;
		}

		// don't accept any more connections (only one client can connect a at time)
		close(socketFD);
		socketFD = -1;

		log("Client connected");
		fcntl(clientFD, F_SETFL, O_NONBLOCK);
		lastClientConnectionTime = system_clock::now();
		system_clock::time_point lastReceiveTime = system_clock::now();

		while (serverRunning) {
			this_thread::sleep_for(std::chrono::milliseconds(5));
			checkResponsiveness(lastReceiveTime);

			unsigned char prefixBuffer[PREFIX_SIZE];
			ssize_t bytesRead = read(clientFD, &prefixBuffer, PREFIX_SIZE);
			if (bytesRead == -1 && errno == EAGAIN) {
				continue;
			}
			if (bytesRead != PREFIX_SIZE) {
				close(clientFD);
				clientFD = -1;
				break;
			}

			auto size = static_cast<size_t>(ByteConverter::BytesToInt(prefixBuffer));
			char messageBuffer[size];
			bytesRead = read(clientFD, messageBuffer, size);
			if (bytesRead == -1 && errno == EAGAIN) {
				continue;
			}
			if (bytesRead != size) {
				close(clientFD);
				clientFD = -1;
				break;
			}

			lastReceiveTime = system_clock::now();
			decoder->decode(messageBuffer);
		}

		log("Client disconnected");
		onClientDisconnected();
	}
}

void Communication::checkResponsiveness(system_clock::time_point lastReceiveTime)
{
	currentTime = system_clock::now();
	std::chrono::duration<double, std::milli> timeSinceLastUpdate = currentTime - lastReceiveTime;
	bool timeIsAboveThreshold = timeSinceLastUpdate >= RESPONSIVENESS_THRESHOLD;

	if (clientResponsive && timeIsAboveThreshold) {
		log("Client unresponsive (stopping motor)");
		clientResponsive = false;
		_action->getMotorEffector(AADCCar::MAIN_MOTOR)->setValue(0, 0);
	} else if (!clientResponsive && !timeIsAboveThreshold) {
		log("Client responding again");
		clientResponsive = true;
	}
}

void Communication::onClientDisconnected()
{
	_action->getMotorEffector(AADCCar::MAIN_MOTOR)->setValue(0, 0);
	_action->getServoDriveEffector(AADCCar::STEERING_SERVO)->setValue(0, 0);

	for (auto &lightConfig : _carMetaModel->getLightConfigs()) {
		_action->getLightEffector(lightConfig->getEffectorName())->setValue(false, 0);
	}
}

void Communication::log(std::string message)
{
	cout << "[Communication] " << message << endl;
}

void Communication::error(std::string message)
{
	cerr << "[Communication] " << message << " failed: " << strerror(errno) << endl;
}