 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "KIFlyTo.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"
#include "KIRecoverVision.h"


KIFlyTo::KIFlyTo(DronePosition checkpointP, 
		double stayTime,
		double maxControlFactorP,
		double initialReachedDistP,
		double stayWithinDistP,
		bool hovering)
{
	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;
	initialReachedDist = initialReachedDistP;
	stayWithinDist = stayWithinDistP;

	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;
	
	targetSet = false;

	isCompleted = false;

	char buf[200];
	snprintf(buf,200,"goto %.2f %.2f %.2f %.2f", checkpointP.pos[0], checkpointP.pos[1], checkpointP.pos[2], checkpointP.yaw);
	command = buf;

	hoverOnly = hovering;

	KIRecoverVision::currentFlyToCommand(checkpointP);

}


KIFlyTo::~KIFlyTo(void)
{
}


bool KIFlyTo::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(!targetSet)
	{
		controller->setTarget(checkpoint);

		manualScaleKeypoint = fabs(checkpoint.pos[2] - statePtr->z) >= 0.1; // If the zDist between the planned waypoint and the current position is big enough, send manual keypoint messages.
		std::cout << "Distance between current location and goal: " << fabs((double) checkpoint.pos[2] - statePtr->z) << std::endl;
		// std::cout << "first: " << checkpoint.pos[2] << " second " << statePtr->z << std::endl;


		if(manualScaleKeypoint)
		{
			std::cout << "Do manual Scale Keypoint" << std::endl;
			node->manualScaleActivate(true);
		}

	}
	targetSet = true;

	// target reached?
	if(!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs)
	{
		printf("checkpoint done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		ControlCommand cmd = controller->update(statePtr);
		if(hoverOnly)
		{
			cmd.roll = 0.0;
			cmd.pitch = 0.0;
		}
		node->sendControlToDrone(cmd);

		if(manualScaleKeypoint)
		{
			node->manualScaleActivate(false);

			// Send it only once...
			manualScaleKeypoint = false;
		}

		return true;
	}


	// get target dist:
	TooN::Vector<3> diffs = TooN::makeVector(
			statePtr->x - checkpoint.pos[0],
			statePtr->y - checkpoint.pos[1],
			statePtr->z - checkpoint.pos[2]);

	// double diffYaw = statePtr->yaw - checkpoint.yaw;
	double diffYaw = controller->getYawDiff(checkpoint.yaw, statePtr->yaw);
	double diffDistSquared = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

	int yawThres = 50;

	if(hoverOnly)
	{
		diffDistSquared = 0.0;
	}

	// if not reached yet, need to get within small radius to count.
	if(!reached && diffDistSquared < initialReachedDist * initialReachedDist && diffYaw*diffYaw < yawThres)
	{
		reached = true;
		reachedAtClock = getMS();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if(reached && (diffDistSquared > stayWithinDist * stayWithinDist || diffYaw*diffYaw > yawThres))
	{
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	ControlCommand cmd = controller->update(statePtr);
	if(hoverOnly)
	{
		cmd.roll = 0.0;
		cmd.pitch = 0.0;
	}
	node->sendControlToDrone(cmd);
	return false;	// not done yet (!)
}
