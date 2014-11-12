#ifndef _GAZEBO_ANTZ_HH_
#define _GAZEBO_ANTZ_HH_
#include <gazebo/gazebo.hh>
#include <math.h>
#include <vector>

#define ANTZ_LEN					0.2 // meter(s)
#define ANTZ_HGT					0.1 // meter(s)
#define WORLD_LEN					(50 * ANTZ_LEN)
#define NEST_X						(-0.3 * WORLD_LEN)
#define NEST_Y						0
#define FOOD_X						(0.3 * WORLD_LEN)
#define FOOD_Y						0

#define ANTZ_COUNT					70
#define TARGET_COUNT                2 // including nest
#define SPAWN_INTERVAL				5 // sec(s)
#define COMM_RANGE					(10 * ANTZ_LEN)
#define DETECT_RANGE				(1.1 * ANTZ_LEN)
#define SPEED						ANTZ_LEN
#define CHANGE_PROB					0.03
#define EXPLORE_TIME				10 // sec(s) ("step count")
#define RETRIEVE_TIME				30 // sec(s)
#define AVOID_TIME					0.5 // sec(s)
#define AVOID_ANGLE					(M_PI * 0.25) // radians

#define SIGN_THR					3
#define IDENT_THR					0
#define	WAIT_THR					30 // sec(s)

#define ANTZ(id, field)             AntzInfo::antz[id].field

namespace gazebo {
    struct AntzInfo { // globally shared info
        int target; // 0 - nest, 1 - food
        int frequency; // visited frequency
        int count; // number of walkers round
        std::vector<int> close;
        std::vector<double> attractive;
        const math::Vector3* position; // read-only position
        AntzInfo(const math::Vector3* pos);
        
        static std::vector<AntzInfo> antz;
        static const math::Vector3 targetPos[TARGET_COUNT];
    };
}

#endif