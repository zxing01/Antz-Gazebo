#include "Antz.hh"
#include <limits>

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////////////////
std::vector<AntzInfo> AntzInfo::antz;
const math::Vector3 AntzInfo::targetPos[TARGET_COUNT] = {
    math::Vector3(NEST_X, NEST_Y, ANTZ_HGT / 2),
    math::Vector3(FOOD_X, FOOD_Y, ANTZ_HGT / 2)
};

//////////////////////////////////////////////////////////////////////////////////////////
AntzInfo::AntzInfo(const math::Vector3* pos):
target(1),
frequency(0),
count(0),
close(TARGET_COUNT, -1),
attractive(TARGET_COUNT, std::numeric_limits<double>::infinity()),
position(pos) {}
