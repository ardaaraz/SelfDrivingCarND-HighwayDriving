#ifndef param_h
#define param_h

/** @brief Max. speed for ego vehicle in m/s */
#define TARGET_VEL (49.5*0.44704)

/** @brief Number of lanes for the highway */
#define MAX_LANE 3

/** @brief Number of waypoints for trajectory */
#define WAY_POINT_NUM 50 

/** @brief Acceleration Limit for the highway */
#define ACC_LIM 10

/** @brief Safe distance to avoid collision */
#define SAFE_DIST 40

/** @brief Safe distance for lane change */
#define CLEAR_DIST 100

/** @brief Penalty for slow lane */
#define VELOCITY_COST_GAIN 1e3

/** @brief Penalty for distance between vehicles lane */
#define DIST_COST_GAIN 5e4

/** @brief Penalty for distance from center lane */
#define CENTER_COST_GAIN 1e4

#endif