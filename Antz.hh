#ifndef _GAZEBO_ANTZ_HH_
#define _GAZEBO_ANTZ_HH_
#include <math.h>

#define ANTZ_LEN					0.2 // meter(s)
#define ANTZ_HGT					0.1 // meter(s)
#define WORLD_LEN					(50 * ANTZ_LEN)
#define NEST_X						(-0.3 * WORLD_LEN)
#define NEST_Y						0
#define FOOD_X						(0.3 * WORLD_LEN)
#define FOOD_Y						0

#define ANTZ_COUNT					70
#define SPAWN_INTERVAL				2 // sec(s)
#define COMM_RANGE					(10 * ANTZ_LEN)
#define SPEED						ANTZ_LEN
#define CHANGE_PROB					0.01
#define EXPLORE_TIME				4 // sec(s) ("step count")
#define RETRIEVE_TIME				30 // sec(s)
#define AVOID_TIME					0.5 // sec(s)
#define AVOID_ANGLE					(M_PI * 0.25) // radians
#define AVOID_RANGE					(1.05 * ANTZ_LEN)
#define TARGET_RANGE				(2 * ANTZ_LEN)

#define SIGN_THR					3
#define IDENT_THR					0
#define	WAIT_THR					30 // sec(s)

#endif