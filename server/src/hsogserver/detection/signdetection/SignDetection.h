#include <detection/RoadSign.h>
#include <perception/impl/RoadSignPerceptor.h>

namespace taco
{
class SignDetection
{
  public:
	static const RoadSign buildRoadSign(taco::IRoadSignPerceptor::ConstPtr perceptor);
};
}
