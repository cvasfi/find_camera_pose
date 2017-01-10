/*
 * KIRecoverVision.cpp
 *
 *  Created on: 04.05.2015
 *      Author: lukas
 */

#include "KIRecoverVision.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KIRecoverVision::KIRecoverVision(KIProcedure *lastProcedure) {

	called = false;
	last = lastProcedure;
	ready = false;
	targetSet = false;
}

KIRecoverVision::~KIRecoverVision() {

	if(last != 0)
	{
		delete last;
		last = 0;
	}
}

DronePosition KIRecoverVision::lastGoal;
DronePosition KIRecoverVision::currentGoal;

void KIRecoverVision::currentFlyToCommand(DronePosition goal)
{
	lastGoal = currentGoal;
	currentGoal = goal;
	// TODO lsd: what happens, if vision is lost, before lastGoal is initialised...???? (probably a problem...)
}

bool KIRecoverVision::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(statePtr->ptamState == PTAM_GOOD || statePtr->ptamState == PTAM_BEST && !ready)
	{
		ROS_INFO("vision recovered :)");
		ready = true;
		if(called)
		{
			controller->agressiveness = lastAggressiveness;
		}
	}
	if(!ready)
	{
		if(statePtr->ptamState == PTAM_IDLE)
		{
			node->sendControlToDrone(node->hoverCommand);
			return false;
		}
		if(!called)
		{
			ROS_INFO("Trying to recover vision.");
			called = true;
			firstTime = ros::Time::now();
			lastAggressiveness = controller->agressiveness;
		}

		double timeSince = (ros::Time::now() - firstTime).toSec();
		if(timeSince <= 20.0)
		{
			ROS_DEBUG("Waiting for vision to recover.");
			// wait for 20s, to recover tracking. Then try to go to the last valid Position...
			node->sendControlToDrone(node->hoverCommand);
			return false;
		}


		if(!targetSet)
		{
			ROS_INFO("Sending last goal, to recover vision");
			controller->setTarget(lastGoal);
			targetSet = true;
		}
		// set down aggressiveness
		controller->agressiveness = 0.2;

		node->sendControlToDrone(controller->update(statePtr));

		return false;
	}else
	{
		ROS_INFO("Returning to current goal...");
		if(last == 0 || true) // TODO: should be changed someday maybe
		{
			return true;
		}
		if(targetSet)
		{
			controller->setTarget(currentGoal);
			targetSet = false;
		}
		return last->update(statePtr);
	}



}
