/*
 * KITakeOff.cpp
 *
 *  Created on: 08.05.2015
 *      Author: lukas
 */

#include "KITakeOff.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KITakeOff::KITakeOff() {

	sentTakeOff = false;

}

KITakeOff::~KITakeOff() {

}

bool KITakeOff::update(const tum_ardrone::filter_stateConstPtr statePtr) {

	if(!sentTakeOff)
	{
		node->sendTakeoff();
	}
	sentTakeOff = true;

	node->sendControlToDrone(node->hoverCommand);

	if(statePtr->droneState == 3 || statePtr->droneState == 4)
	{
		// ready...

		return true;
	}

	return false;

}
