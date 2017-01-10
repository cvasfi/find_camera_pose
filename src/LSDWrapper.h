/*
 * LSDWrapper.h
 *
 *  Created on: 09.04.2015
 *      Author: Lukas von Stumberg
 */

#ifndef TUM_ARDRONE_SRC_LSD_STATEESTIMATION_LSDWRAPPER_H_
#define TUM_ARDRONE_SRC_LSD_STATEESTIMATION_LSDWRAPPER_H_

#include "PTAMWrapper.h"
#include "lsd_slam_viewer/extendedTrackedFrameMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include <Eigen/Geometry>
#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <map>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include "ModifiedOctomap.h"

struct InputPointDense {
	float idepth;
	float idepth_var;
	unsigned char color[4];
};

struct GraphFramePose {
	int id;
	float camToWorld[7];
};
struct GraphConstraint
{
	int from;
	int to;
	float err;
};

struct DroneCommand {
public:
	DroneCommand(octomap::point3d position, double yaw);
	octomap::point3d position;
	double yaw;
};


typedef boost::function<void(std::string)> PublishCommandFunction;
typedef boost::function<void()> EmptyFunction;

class LSDWrapper;
class PTAMWrapper;

typedef std::vector<std::vector<std::vector<int> > > MarkingArray;

struct InterestingPoint {
public:
	InterestingPoint(Sophus::Sim3f camToParent, Sophus::Sim3f firstParentToWorld, int keyframeId);

	Sophus::Sim3f camToParent;
	int keyframeId;

	bool didDiscover;
	int numStarDiscoveries;
	double sumDiscoverDist;

	void markPointsInSight(MarkingArray *marked, int markNumber,
			octomap::OcTree *tree, LSDWrapper *wrapper, int threshold, int hDisp = 0);
	double computeNextStarDiscovery(MarkingArray *marked, int markNumber,
			double currentYaw, LSDWrapper *wrapper, octomap::OcTree *tree, std::vector<DroneCommand> *commands);

	double computeNextStarDiscoveryOtherTactic(MarkingArray *marked, int markNumber,
				double currentYaw, LSDWrapper *wrapper, octomap::OcTree *tree, std::vector<DroneCommand> *commands);

	static void markPointsInSight(MarkingArray *marked, int markNumber,
				octomap::OcTree *tree, LSDWrapper *wrapper, octomap::point3d pos, int threshold);
	static double computeNextStarDiscovery(MarkingArray *marked, int markNumber,
				double currentYaw, LSDWrapper *wrapper, octomap::OcTree *tree, std::vector<DroneCommand> *commands, octomap::point3d pos);


	static double computeNextStarDiscoveryOtherTactic(MarkingArray *marked, int markNumber,
					double currentYaw, LSDWrapper *wrapper, octomap::OcTree *tree, std::vector<DroneCommand> *commands, octomap::point3d pos);

	static void markPointsInSightOld(MarkingArray* marked, int markNumber,
			octomap::OcTree* tree, LSDWrapper* wrapper, octomap::point3d myPos);


	Sophus::Sim3f firstParentToWorld;

	static double getAngle(octomap::point3d point)
	{
		if (point.x() < 0)
		{
			return 360 - (atan2(point.x(), point.z()) * -1 * 180 / M_PI);
		}
		else
		{
			return atan2(point.x(), point.z()) * 180 / M_PI;
		}
	}

	octomap::point3d getOwnPosition(LSDWrapper* wrapper);
};

class LSDKeyframe {
public:

	LSDKeyframe();

// 	void setCamToWorld(Sophus::Sim3f camToWorld);

	std::auto_ptr<InputPointDense> points;
	int id;

	std::vector<Sophus::Sim3f>* getPoses();

	void insertedToOctomap();
	double getDistanceToLastInserted();
	Eigen::Vector3f getVectorToLastInserted();
	bool wasInserted();

	const Sophus::Sim3f& getCamToWorld() const {
		return camToWorld;
	}

	void setCamToWorld(const Sophus::Sim3f& camToWorld) {
		camToWorldSet = true;
		this->camToWorld = camToWorld;
	}

	bool isCamToWorldSet() const {
		return camToWorldSet;
	}

	std::vector<std::pair<int, float> > constraints;

private:

	bool camToWorldSet;
	Sophus::Sim3f camToWorld;
	std::vector<Sophus::Sim3f> poses;

	Sophus::Sim3f poseInsertedToOctomap;
	bool inserted;

};


class LSDWrapper {
public:

	enum CurrentControl{LOOK_AROUND, STAR_DISCOVERY, PATH};

	LSDWrapper(PTAMWrapper *ptamWrapper, PublishCommandFunction publish, EmptyFunction startLSDFunction);
	virtual ~LSDWrapper();

	void extendedPose(lsd_slam_viewer::extendedTrackedFrameMsg msg);

	void keyframe(lsd_slam_viewer::keyframeMsg msg);

	void keyframeGraph(lsd_slam_viewer::keyframeGraphMsg msg);

	void doOctomap(std_msgs::Empty msg);

	Eigen::Vector3f convertToAutopilot(Sophus::Sim3f sim3,
			Eigen::Vector3f *rpy = 0);
	Eigen::Vector3f convertToAutopilot(octomap::point3d& point);

	LSDKeyframe* getKeyframeById(int id) {
		return keyframeById[id];
	}

	void addNewCommands(std_msgs::Empty msg);

	void addNewCommandsInOwnThread(std_msgs::Empty msg);


	std::vector<LSDKeyframe*>& getKeyframes() {
		return keyframes;
	}

	octomap::point3d eigenVecToOcto(const Eigen::Vector3f& camPos);

	octomap::OcTreeKey getOctomapMinKey(octomap::OcTree &tree);
	octomap::OcTreeKey getOctomapMaxKey(octomap::OcTree &tree);
	void traverseOctomap(octomap::OcTree &tree, boost::function<void(int,int,int, octomap::OcTreeKey, octomap::OcTree&)> callback);

	octomap::point3d convertCameraToDroneCenter(Sophus::Sim3f pos);

	// void iterateTree(octomap::OcTree &tree, )

	void analyseOctree(std::string filename);

	void insertPointCloudWithMisses(octomap::OcTree &tree, const octomap::Pointcloud& scan, octomap::Pointcloud& scanMiss, const octomap::point3d& sensor_origin,
            double maxrange, bool lazy_eval, bool discretize, double *insertionTime = 0, double *setCreationTime = 0);

	int lastParentId;
	int previousParentId;
	Sophus::Sim3f lastParentToWorld;
	Sophus::Sim3f currentPositionOffset;

	std::vector<LSDKeyframe*> keyframes;
	std::map<int, LSDKeyframe*> keyframeById;

	std::vector<GraphConstraint> constraints;
	pthread_mutex_t constraintsMutex;

private:

	void saveOctomap(octomap::OcTree &octree, std::string name, int number = -1);

	void createUpdatedOctomap(int distXZ, int distY, octomap::OcTree *newTree, int distYObstacle = 3);

	void generateOctomap(int sparsify1 = 1, int sparsify2 = 2);

	PTAMWrapper *ptamWrapper;

	Sophus::Sim3f deserializeSim3(boost::array<float, 7> pos);
	Sophus::Sim3f deserializeSim3(float *pos);

	TooN::SE3<> sophusToToonSE3(Sophus::SE3f s);
	void testRayCast(const octomap::point3d& direction, bool treat);

	void addMoveCommand(std::stringstream& commands, double angle,
			octomap::point3d& end);
	void convertToFreeTree(octomap::OcTree& tree, std::string name);

	void generateLookAroundCommands(std::vector<DroneCommand>& output, int number, octomap::point3d position, double currentYaw = 0.0);

	std::vector<InterestingPoint> points;

	void createBoundingOctree();

	// std::vector<std::vector<Sophus::Sim3f>> allPoses;

	int width;
	int height;

	octomap::ModifiedOctomap tree;
	octomap::OcTree freeTree, freeTreeBigger;

	Sophus::Sim3f lastCamToWorld;

	Sophus::Sim3f lastCamToWorldWithoutTransform;

	double lastYaw;

	Sophus::Sim3f lastCamToParent;

	bool camDataSet;
	float fx, fy, fxi, fyi, cx, cy, cxi, cyi;

	std::vector<InterestingPoint> interestingPoints;

	std::vector<DroneCommand> commands;
	PublishCommandFunction publish;
	EmptyFunction startLSD;


	std::fstream positionPlotFile, positionOffsetFile;

	CurrentControl currControl;

	bool takenOffOnce;
	bool justTakingOff;

	void visualizeMarkedPointsCallback(int x, int y, int z, octomap::OcTreeKey nk, octomap::OcTree &octomap, octomap::OcTree *newOctomap, std::vector<std::vector<std::vector<int> > > *marked);
	void visualizeMarkedPoints(octomap::OcTree* testTree,
			std::vector<std::vector<std::vector<int> > >& marked, octomap::OcTree &originalTree);

	void markInterestingPointsCallback(int x, int y, int z, octomap::OcTreeKey nk, octomap::OcTree &octomap, MarkingArray *interesting, MarkingArray *uninteresting);

	bool floodFill(int x, int y, int z, MarkingArray &marked, MarkingArray &flooded, int currMarker);

	void visualizeCommands(octomap::OcTree &tree, octomap::OcTree &newTree, std::vector<DroneCommand> &commands);
	void printCommands(std::vector<DroneCommand> &commands, std::string name);
	void copyOccupiedPointsCallback(int x, int y, int z, octomap::OcTreeKey nk, octomap::OcTree &octomap, octomap::OcTree *newOctomap);

	bool generatePathMoveCommands(octomap::point3d start, octomap::point3d end, std::vector<DroneCommand> &commands, octomap::OcTree &octomap);

	int callNumber;

	template<typename A> void initOctomapArray(octomap::OcTree &tree, std::vector<std::vector<std::vector<A> > > &array, A init);

	void addCommandsToQueue(std::vector<DroneCommand> commands);

	bool wentToNewIntPoint;
	bool addingCommands;

	Eigen::Vector3f nextCenterPose;
	double nextCenterYaw;

	void paintFree(octomap::OcTree &t, octomap::OcTreeKey pos, int dx, int dy, int dz);


};

template<typename A>
inline void LSDWrapper::initOctomapArray(octomap::OcTree& tree,
		std::vector<std::vector<std::vector<A> > >& array, A init) {

	octomap::OcTreeKey kMin = getOctomapMinKey(tree);
	octomap::OcTreeKey kMax = getOctomapMaxKey(tree);

	int maxX = kMax.k[0]-kMin.k[0];
	int maxY = kMax.k[1]-kMin.k[1];
	int maxZ = kMax.k[2]-kMin.k[2];

	array.resize(maxX+1);
	for(int x = 0; x < maxX+1; ++x)
	{
		array[x].resize(maxY+1);
		for(int y = 0; y < maxY+1; ++y)
		{
			array[x][y].resize(maxZ+1);
			for(int z = 0; z < maxZ+1; ++z)
			{
				array[x][y][z] = init;
			}
		}
	}

}

class FileCloser
{
public:
	FileCloser();
	~FileCloser();

	std::fstream *file;
};
class TimeMeasuring
{
public:
	TimeMeasuring(std::string name);
	void end();
	static bool init;
private:
	std::string name;
	ros::Time start;

	static std::fstream file;
	static FileCloser closer;
};

octomap::OcTreeKey operator+( const octomap::OcTreeKey& v1, const octomap::OcTreeKey& v2 );
octomap::OcTreeKey operator-( const octomap::OcTreeKey& v1, const octomap::OcTreeKey& v2 );
std::ostream& operator<<(std::ostream& os, const octomap::OcTreeKey& obj);

#endif /* TUM_ARDRONE_SRC_LSD_STATEESTIMATION_LSDWRAPPER_H_ */
