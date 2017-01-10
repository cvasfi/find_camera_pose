/*
 * KIRecoverVision.h
 *
 *  Created on: 04.05.2015
 *      Author: lukas
 */

#ifndef TUM_ARDRONE_SRC_AUTOPILOT_KI_KIRECOVERVISION_H_
#define TUM_ARDRONE_SRC_AUTOPILOT_KI_KIRECOVERVISION_H_

#include "KIProcedure.h"
#include "ros/ros.h"

class KIRecoverVision : public KIProcedure {

public:
	KIRecoverVision(KIProcedure *lastProcedure);
	virtual ~KIRecoverVision();

	static void currentFlyToCommand(DronePosition goal);

	bool update(const tum_ardrone::filter_stateConstPtr statePtr);

	enum {PTAM_IDLE = 0, PTAM_INITIALIZING = 1, PTAM_LOST = 2, PTAM_GOOD = 3, PTAM_BEST = 4, PTAM_TOOKKF = 5, PTAM_FALSEPOSITIVE = 6} PTAMStatus;

private:

	static DronePosition lastGoal;
	static DronePosition currentGoal;
	ros::Time firstTime;
	bool called;
	double lastAggressiveness;
	KIProcedure *last;
	bool ready;
	bool targetSet;

};

#endif /* TUM_ARDRONE_SRC_AUTOPILOT_KI_KIRECOVERVISION_H_ */
