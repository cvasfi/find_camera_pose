/*
 * KITakeOff.h
 *
 *  Created on: 08.05.2015
 *      Author: lukas
 */

#ifndef TUM_ARDRONE_SRC_AUTOPILOT_KI_KITAKEOFF_H_
#define TUM_ARDRONE_SRC_AUTOPILOT_KI_KITAKEOFF_H_

#include "KIProcedure.h"
#include "ros/ros.h"

class KITakeOff : public KIProcedure {
public:
	KITakeOff();
	virtual ~KITakeOff();

	bool update(const tum_ardrone::filter_stateConstPtr statePtr);

private:
	bool sentTakeOff;

};

#endif /* TUM_ARDRONE_SRC_AUTOPILOT_KI_KITAKEOFF_H_ */
