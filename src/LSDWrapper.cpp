/*
 * LSDWrapper.cpp
 *
 *  Created on: 09.04.2015
 *      Author: lukas
 */

#include "LSDWrapper.h"
#include <Eigen/Geometry>
#include "TooN/se3.h"
#include "sophus/sim3.hpp"
#include "SophusUtil.h"
#include <math.h>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>

void LSDWrapper::createBoundingOctree() {
	// octomap::point3d start(-0.231-0.35, -1.336-0.35, 0.0);
	// octomap::point3d end(2.22+0.35, 1.653+0.35, 2.5);
	octomap::point3d start(-0.231 - 0.35, -1.336 - 0.35, 0.0);
	octomap::point3d end(2.22 + 0.35, 1.653 + 0.35, 2.5);
	octomap::point3d floorStart(start.x(), start.y(), -0.5);

	double resolution = 0.05;
	octomap::OcTree *tree = new octomap::OcTree(resolution);
	octomap::OcTree *withoutFloor = new octomap::OcTree(resolution);

	tree->updateNode(start, true, true);
	tree->updateNode(end, true, true);
	// tree->updateNode(floorEnd, false, true);

	withoutFloor->updateNode(start, true, true);
	withoutFloor->updateNode(end, true, true);
	// withoutFloor->updateNode(floorEnd, false, true);

	octomap::OcTreeKey kMin = tree->coordToKey(start);
	octomap::OcTreeKey kMax = tree->coordToKey(end);
	// octomap::OcTreeKey kMin = getOctomapMinKey(*tree);
	// octomap::OcTreeKey kMax = getOctomapMaxKey(*tree);
	// octomap::OcTreeKey floorHeight = tree->coordToKey(end);
	octomap::OcTreeKey nextStart = tree->coordToKey(floorStart);
	std::cout << kMin << std::endl;
	std::cout << nextStart << std::endl;

	for (int x = nextStart.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = nextStart.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = nextStart.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;

				if (x == kMin[0] || x == kMax[0] || y == kMin[1] || y == kMax[1]
						|| z == kMin[2] || z == kMax[2]) {
					tree->updateNode(nk, true, true);
				} else {
					tree->updateNode(nk, false, true);
				}

				if (x == kMin[0] || x == kMax[0] || y == kMin[1] || y == kMax[1]
						|| z == kMax[2]) {
					withoutFloor->updateNode(nk, true, true);
				} else {
					withoutFloor->updateNode(nk, false, true);
				}
				// callback(x - kMin.k[0], y - kMin.k[1], z - kMin.k[2], nk, tree);

			}
		}
	}

	tree->updateInnerOccupancy();
	withoutFloor->updateInnerOccupancy();

	tree->writeBinary("boundingBox.bt");
	withoutFloor->writeBinary("boundingBoxWithoutFloor.bt");

}

LSDWrapper::LSDWrapper(PTAMWrapper* ptamWrapper, PublishCommandFunction publish,
		EmptyFunction startLSDFunction) :
		tree(0.05), camDataSet(false), freeTree(0.05), freeTreeBigger(0.05), callNumber(
				0), commands(), publish(publish), wentToNewIntPoint(false), justTakingOff(
				false), addingCommands(false), startLSD(startLSDFunction), lastYaw(
				0), constraints(), previousParentId(-1), constraintsMutex(
				PTHREAD_MUTEX_INITIALIZER), takenOffOnce(false) {

	this->ptamWrapper = ptamWrapper;
	this->currentPositionOffset = Sophus::Sim3f();
	width = 0;
	height = 0;

	positionPlotFile.open("Flight 1/positionsWithAndWithoutOffset.txt",
			std::fstream::out | std::fstream::trunc);
	positionOffsetFile.open("Flight 1/positionOffsets.txt",
			std::fstream::out | std::fstream::trunc);

	/*std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(1.0, 0.0, 0.0)) << std::endl;
	 std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(0.0, 1.0, 0.0)) << std::endl;
	 std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(0.5, 0.5, 0.0)) << std::endl;
	 std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(-1.0, 0.0, 0.0)) << std::endl;
	 std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(0.0, -1.0, 0.0)) << std::endl;
	 std::cout << "AngleTest!!! " << std::endl << InterestingPoint::getAngle(octomap::point3d(-0.5, -0.5, 0.0)) << std::endl;*/

	// TEST CODE
	/*analyseOctree("Flight 2015-09-10-18-33-50.bag/Run 2/tree.bt");
	 analyseOctree("Flight 2015-09-10-18-33-50.bag/Run 3/tree.bt");*/

	// createBoundingOctree();
	//analyseOctree("Flight 2015-09-13-17-04-59.bag/Run 1/tree.bt");

	/*analyseOctree("Flight\ 2015-09-13-20-32-44.bag/Run\ 2/tree.bt");
	 analyseOctree("Flight\ 2015-09-13-20-32-44.bag/Run\ 3/tree.bt");
	 analyseOctree("Flight\ 2015-09-13-20-32-44.bag/Run\ 4/tree.bt");*/

}

LSDWrapper::~LSDWrapper() {

	positionPlotFile.close();
	positionOffsetFile.close();

	for (int i = 0; i < keyframes.size(); ++i) {
		if (keyframes[i]) {
			delete keyframes[i];
		}
	}

}

void LSDWrapper::extendedPose(lsd_slam_viewer::extendedTrackedFrameMsg msg) {
	Sophus::Sim3f camToParent = deserializeSim3(msg.camToParent);
	Sophus::Sim3f parentToWorld = deserializeSim3(msg.parentToWorld);
	int parentId = msg.parentId;

	if (msg.trackingLost) {
		ptamWrapper->HandleFrame(TooN::SE3<>(), true, ros::Time(msg.time),
				this);
	} else {
		if (parentId != lastParentId) {
			// currentPositionOffset =  currentPositionOffset * lastParentToWorld * parentToWorld.inverse();
			// currentPositionOffset = parentToWorld;
			previousParentId = lastParentId; // When the parent changes, also update the previousParentId...

		} else {
			bool equal = parentToWorld.translation()
					== lastParentToWorld.translation();
			equal = equal
					&& parentToWorld.rotationMatrix()
							== lastParentToWorld.rotationMatrix();
			// if(parentToWorld != lastParentToWorld)
			if (!equal) {
				// newPositionOffset * parentToWorld  == oldPositionOffset * lastParentToWorld
				// => new PositionOffset = oldPositionOffset * lastParentToWorld * inv(parentToWorld)
				currentPositionOffset = currentPositionOffset
						* lastParentToWorld * parentToWorld.inverse();
				// std::cout << "should happen." << std::endl;
			}
		}
		Sophus::Sim3f camToWorld = currentPositionOffset * parentToWorld
				* camToParent;

		lastCamToWorldWithoutTransform = parentToWorld * camToParent;
		lastCamToWorld = camToWorld;
		lastCamToParent = camToParent;
		lastParentId = msg.parentId;

		Sophus::Vector3f rpy;
		convertToAutopilot(lastCamToWorldWithoutTransform, &rpy);
		lastYaw = rpy.z();

		LSDKeyframe *frame = keyframeById[parentId];
		if (frame == 0) {
			frame = new LSDKeyframe();
			frame->setCamToWorld(parentToWorld);
			keyframeById[parentId] = frame;
		}
		frame->getPoses()->push_back(camToParent);

		ptamWrapper->HandleFrame(
				sophusToToonSE3(lsd_slam::se3FromSim3(camToWorld)), false,
				ros::Time(msg.time), this);

	}

	positionPlotFile << lastCamToWorldWithoutTransform.translation().x() << ' '
			<< lastCamToWorldWithoutTransform.translation().y() << ' '
			<< lastCamToWorldWithoutTransform.translation().z() << ' '
			<< lastCamToWorld.translation().x() << ' '
			<< lastCamToWorld.translation().y() << ' '
			<< lastCamToWorld.translation().z() << std::endl;

	positionOffsetFile << currentPositionOffset.rotationMatrix() << ' '
			<< currentPositionOffset.translation() << std::endl;

	// memcpy(msg.camToParent.data(), kf->pose->thisToParent_raw.cast<float>().data(), sizeof(float) * 7);

	lastParentId = parentId;
	lastParentToWorld = parentToWorld;
}

Sophus::Sim3f LSDWrapper::deserializeSim3(boost::array<float, 7> pos) {
	return deserializeSim3(pos.data());
}

Sophus::Sim3f LSDWrapper::deserializeSim3(float* pos) {
	Sophus::Sim3f ret;
	memcpy(ret.data(), pos, sizeof(float) * 7);
	return ret;
}

Eigen::Vector3f LSDWrapper::convertToAutopilot(Sophus::Sim3f sim3,
		Eigen::Vector3f *rpy) {
	// TODO: maybe multiply with currentPositionOffset first.
	TooN::Vector<6> res = ptamWrapper->transformCoords(
			sophusToToonSE3(
					lsd_slam::se3FromSim3(currentPositionOffset * sim3)));
	Eigen::Vector3f ret(res[0], res[1], res[2]);
	if (rpy != 0) {
		rpy->x() = res[3];
		rpy->y() = res[4];
		rpy->z() = res[5];
	}
	return ret;
}

Eigen::Vector3f LSDWrapper::convertToAutopilot(octomap::point3d& point) {
	Sophus::Sim3f sim3;
	sim3.translation() = Eigen::Vector3f(point.x(), point.y(), point.z());
	Eigen::Vector3f pos = convertToAutopilot(sim3);
	return pos;
}

octomap::point3d LSDWrapper::eigenVecToOcto(const Eigen::Vector3f& camPos) {
	octomap::point3d sensor(camPos.x(), camPos.y(), camPos.z());
	return sensor;
}

void LSDWrapper::generateOctomap(int sparsify1, int sparsify2) {

	std::cout << "Octomap values " << tree.getProbHit() << " "
			<< tree.getProbMiss() << " " << tree.getOccupancyThres()
			<< std::endl;

	float my_scale = lastCamToWorld.scale(); // TODO lsd: maybe add maximum-likelyhood scaling factor...
	// float my_scaledTH = exp10(-2.5);
	float my_scaledTH = exp10(-3.0);
	float my_scaledTH2 = exp10(-2.0);
	int my_minNearSupport = 7;
	float my_absTH = exp10(-1.0);
	float my_absTH2 = exp10(-1.0);
	// int sparsify1 = 1;
	// /int sparsify2 = 2;

	int otherMinNearSupport = 9;

	double maxDistToLast = 0.0;
	Eigen::Vector3f sumDist;
	int numDist = 0;

	std::cout << "Number of keyframes: " << keyframes.size() << std::endl;
	long numProcessedPoints = 0;
	long numTotalPoints = 0;

	double maxDist = 0;
	octomap::Pointcloud cloud;
	octomap::Pointcloud cloudMissed;

	double iterationTime = 0.0, setCreationTime = 0.0, insertionTime = 0.0,
			finalizationTime = 0.0;

	for (int i = 0; i < keyframes.size(); ++i) {
		LSDKeyframe *frame = keyframes[i];
		cloud.clear();

		InputPointDense *points = frame->points.get();

		ros::Time t1 = ros::Time::now();

		for (int y = 1; y < height - 1; y+=2) {
			for (int x = 1; x < width - 1; x+=2) {
				numTotalPoints++;
				if (points[x + y * width].idepth <= 0)
					continue;

				float depth = 1 / points[x + y * width].idepth;
				float depth4 = depth * depth;
				depth4 *= depth4;
				bool unaccurate = false;

				if (points[x + y * width].idepth_var * depth4 > my_scaledTH)
					unaccurate = true;

				if (points[x + y * width].idepth_var * depth4 * my_scale
						* my_scale > my_absTH)
					unaccurate = true;

				if (points[x + y * width].idepth_var * depth4 > my_scaledTH2)
					continue;

				if (points[x + y * width].idepth_var * depth4 * my_scale
						* my_scale > my_absTH2)
					continue;

				if (unaccurate) {
					if (sparsify2 > 1 && rand() % sparsify2 != 0)
						continue;
				} else {
					if (sparsify1 > 1 && rand() % sparsify1 != 0)
						continue;
				}

				if (my_minNearSupport > 1) {
					int nearSupport = 0;
					for (int dx = -1; dx < 2; dx++)
						for (int dy = -1; dy < 2; dy++) {
							int idx = x + dx + (y + dy) * width;
							if (points[idx].idepth > 0) {
								float diff = points[idx].idepth - 1.0f / depth;
								if (diff * diff
										< 2 * points[x + y * width].idepth_var)
									nearSupport++;
							}
						}

					if (nearSupport < my_minNearSupport
							|| (unaccurate && nearSupport < otherMinNearSupport))
						continue;
				}

				double idepthVar = points[x + y * width].idepth_var;
				if (idepthVar > my_scaledTH) {
					// std::cout << "Var... " << idepthVar << " . depth: " << depth << std::endl;
				}

				if (unaccurate) {
					depth = 1 / (points[x + y * width].idepth + 0.1);
					depth *= 0.8;
				}

				float posX = (x * fxi + cxi) * depth;
				float posY = (y * fyi + cyi) * depth;
				float posZ = depth;

				if (depth > maxDist) {
					maxDist = depth;
				}

				Eigen::Vector3f vec(posX, posY, posZ);
				Eigen::Vector3f realPos = frame->getCamToWorld() * vec;

				// TODO (lsd): Probably: include estimated scale -> just multiply?
				if (unaccurate) {
					cloudMissed.push_back(realPos.x(), realPos.y(),
							realPos.z());
				} else {
					cloud.push_back(realPos.x(), realPos.y(), realPos.z());
				}
				numProcessedPoints++;
			}
		}

		// std::cout << "Maximal Depth: " << maxDist << std::endl;

		Eigen::Vector3f camPos = frame->getCamToWorld().translation();
		octomap::point3d sensor = eigenVecToOcto(camPos);

		double distToLastOcto = frame->getDistanceToLastInserted();

		maxDistToLast = std::max(maxDistToLast, distToLastOcto);

		if (frame->wasInserted()) {
			numDist++;
			sumDist = sumDist + frame->getVectorToLastInserted();
		}

		frame->insertedToOctomap();

		/*Eigen::Vector3f test(0.0, 0.0, 0.0);
		 Eigen::Vector3f newCamPos = frame->camToWorld * test;
		 octomap::point3d sensor = eigenVecToOcto(newCamPos);*/

		// Time for one dataset: 0.866521s
		// tree.insertPointCloud(cloud, sensor, -1, true, true);
		// Time for one dataset: 3.26365s
		// tree.insertPointCloud(cloud, sensor, -1, true, false);
		iterationTime += (ros::Time::now() - t1).toSec();

		// insertPointCloudWithMisses(tree, cloud, cloudMissed, sensor, -1, true, true, &insertionTime, &setCreationTime);
		tree.insertPointCloudWithMisses(cloud, cloudMissed, sensor, -1, true,
				true, &insertionTime, &setCreationTime);

		ros::Time t2 = ros::Time::now();
		// To remove possible gap at camera position.
		std::vector<Sophus::Sim3f> *allPoses = frame->getPoses();

		double changing = tree.getResolution();
		for (int p = 0; p < allPoses->size(); ++p) {
			Eigen::Vector3f pos = frame->getCamToWorld()
					* (*allPoses)[p].translation();

			// TODO (lsd): Actually the pose here should be converted using the new convertDroneCameraToCenter method...
			if (tree.search(pos.x(), pos.y(), pos.z()) == 0) {
				tree.updateNode(pos.x(), pos.y(), pos.z(), false, true);
			}

			// TODO (lsd): This should be even bigger...
			for (int i = 0; i < 8; ++i) {
				float x = pos.x() - changing;
				float y = pos.y() - changing;
				float z = pos.z() - changing;
				if ((i & 1) == 0) {
					x += 2 * changing;
				}
				if ((i & 2) == 0) {
					y += 2 * changing;
				}
				if ((i & 4) == 0) {
					z += 2 * changing;
				}
				if (tree.search(x, y, z) == 0) {
					tree.updateNode(x, y, z, false, true);
				}
			}

			octomap::OcTreeKey k = tree.coordToKey(
					octomap::point3d(pos.x(), pos.y(), pos.z()));
			for (int x = k[0] - 2; x <= k[0] + 2; ++x) {
				for (int z = k[2] - 2; z <= k[2] + 2; ++z) {
					tree.updateNode(octomap::OcTreeKey(x, k[1], z), false,
							true);
				}
			}

		}

		finalizationTime += (ros::Time::now() - t2).toSec();

		/*for (int i = 0; i < 8; ++i) {
		 float x = camPos.x() - changing;
		 float y = camPos.y() - changing;
		 float z = camPos.z() - changing;
		 if ((i & 1) == 0) {
		 x += 2 * changing;
		 }
		 if ((i & 2) == 0) {
		 y += 2 * changing;
		 }
		 if ((i & 4) == 0) {
		 z += 2 * changing;
		 }
		 tree.updateNode(x, y, z, false, true);
		 }*/

	}

	std::cout << "Number of total points: " << numTotalPoints << std::endl;
	std::cout << "Number of processed points: " << numProcessedPoints
			<< std::endl;

	std::cout << "Times..." << std::endl << "IterationTime: " << iterationTime
			<< std::endl;
	std::cout << "SetCreationTime " << setCreationTime << std::endl
			<< "InsertionTime: " << insertionTime << std::endl;
	std::cout << "FinalizationTime: " << finalizationTime << std::endl;

	tree.updateInnerOccupancy();

	std::cout << "Maximum distance to last octomap: " << maxDistToLast
			<< " num: " << numDist << std::endl << "average: "
			<< sumDist * (1.0 / (double) numDist) << std::endl;

}

TooN::SE3<> LSDWrapper::sophusToToonSE3(Sophus::SE3f s) {
	TooN::SE3<> res;
	res.get_translation()[0] = s.translation()[0];
	res.get_translation()[1] = s.translation()[1];
	res.get_translation()[2] = s.translation()[2];

	Eigen::Matrix<float, 3, 3> matrix = s.rotationMatrix();
	TooN::Matrix<3, 3> mat;
	for (int x = 0; x < 3; ++x) {
		for (int y = 0; y < 3; ++y) {
			mat[x][y] = matrix(x, y);
		}
	}

	res.get_rotation() = mat;
	return res;
}

void LSDWrapper::keyframe(lsd_slam_viewer::keyframeMsg msg) {
	if (!camDataSet) {
		width = msg.width;
		height = msg.height;

		fx = msg.fx;
		fy = msg.fy;
		cx = msg.cx;
		cy = msg.cy;

		fxi = 1 / fx;
		fyi = 1 / fy;
		cxi = -cx / fx;
		cyi = -cy / fy;
		camDataSet = true;
	}

	// TODO (lsd): maybe here there occur concurrency issues
	LSDKeyframe *key;
	if (keyframeById[msg.id] != 0) {
		key = keyframeById[msg.id];
	} else {
		key = new LSDKeyframe;
		keyframeById[msg.id] = key;
	}
	key->setCamToWorld(deserializeSim3(msg.camToWorld));
	key->id = msg.id;
	key->points.reset(new InputPointDense[width * height]);
	memcpy(key->points.get(), msg.pointcloud.data(),
			width * height * sizeof(InputPointDense));

	keyframes.push_back(key);

}

void LSDWrapper::keyframeGraph(lsd_slam_viewer::keyframeGraphMsg msg) {
	GraphFramePose *poses = (GraphFramePose*) msg.frameData.data();
	for (int i = 0; i < msg.numFrames; ++i) {
		// Sophus::Sim3f lastCam = keyframeById[poses[i].id]->camToWorld;
		keyframeById[poses[i].id]->setCamToWorld(
				deserializeSim3(poses[i].camToWorld));

		// std::cout << "scale " << lastCam.scale() - keyframeById[poses[i].id]->camToWorld.scale() << std::endl;

	}

	pthread_mutex_lock(&constraintsMutex);
	for (int i = 0; i < keyframes.size(); ++i) {
		keyframes[i]->constraints.clear();
	}

	GraphConstraint* constraintsIn =
			(GraphConstraint*) msg.constraintsData.data();
	constraints.resize(msg.numConstraints);
	float maxErr = 0;
	for (int i = 0; i < msg.numConstraints; i++) {
		constraints[i].err = constraintsIn[i].err;
		constraints[i].from = constraintsIn[i].from;
		constraints[i].to = constraintsIn[i].to;
		getKeyframeById(constraints[i].from)->constraints.push_back(
				std::make_pair(constraints[i].to, constraints[i].err));
		getKeyframeById(constraints[i].to)->constraints.push_back(
				std::make_pair(constraints[i].from, constraints[i].err));
		if (constraints[i].err > maxErr) {
			maxErr = constraints[i].err;
		}
	}

	pthread_mutex_unlock(&constraintsMutex);

	std::cout << "Maximum constraint error!!!!" << maxErr << std::endl;
}

void LSDWrapper::testRayCast(const octomap::point3d& direction, bool treat) {
	octomap::point3d startPoint = eigenVecToOcto(
			keyframes[3]->getCamToWorld().translation());
	octomap::point3d endPoint;
	tree.castRay(startPoint, /*startPoint + */direction, endPoint, treat, 100);
	std::cout << "Test " << endPoint.distance(startPoint) << std::endl;
}

void LSDWrapper::addMoveCommand(std::stringstream& commands, double angle,
		octomap::point3d& end) {
	Eigen::Vector3f pos = convertToAutopilot(end);
	commands << "gotoraw " << pos.x() << " " << pos.y() << " " << pos.z() << " "
			<< angle << std::endl;
}

octomap::OcTreeKey LSDWrapper::getOctomapMinKey(octomap::OcTree &tree) {

	double x, y, z;
	tree.getMetricMin(x, y, z);
	octomap::point3d min(x, y, z);
	return tree.coordToKey(min);

}

octomap::OcTreeKey LSDWrapper::getOctomapMaxKey(octomap::OcTree &tree) {
	double x, y, z;
	tree.getMetricMax(x, y, z);
	octomap::point3d max(x, y, z);
	return tree.coordToKey(max);
}

void LSDWrapper::traverseOctomap(octomap::OcTree &tree,
		boost::function<
				void(int, int, int, octomap::OcTreeKey, octomap::OcTree&)> callback) {

	octomap::OcTreeKey kMin = getOctomapMinKey(tree);
	octomap::OcTreeKey kMax = getOctomapMaxKey(tree);

	for (int x = kMin.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;

				callback(x - kMin.k[0], y - kMin.k[1], z - kMin.k[2], nk, tree);

			}
		}
	}

}

void LSDWrapper::convertToFreeTree(octomap::OcTree& tree, std::string name) {

	octomap::OcTree *newTree = new octomap::OcTree(tree.getResolution());

	octomap::OcTreeKey kMin = getOctomapMinKey(tree);
	octomap::OcTreeKey kMax = getOctomapMaxKey(tree);

	std::cout << "max " << kMax.k[0] << " " << kMax.k[1] << " " << kMax.k[2]
			<< std::endl << "min " << kMin.k[0] << " " << kMin.k[1] << " "
			<< kMin.k[2] << std::endl;
	std::cout << "number: "
			<< (kMax.k[0] - kMin.k[0]) * (kMax.k[1] - kMin.k[1])
					* (kMax.k[2] - kMin.k[2]) << std::endl;

	for (int x = kMin.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;
				octomap::OcTreeNode *node = tree.search(nk);

				if (node) {
					if (tree.isNodeOccupied(node)) {
						newTree->updateNode(nk, true, true);
					}
				} else {
					newTree->updateNode(nk, false, true);
				}

			}
		}
	}

	newTree->updateInnerOccupancy();

	saveOctomap(*newTree, name);

	delete newTree;

}

void LSDWrapper::createUpdatedOctomap(int distXZ, int distY,
		octomap::OcTree *newTree, int distYObstacle) {

	std::cout << std::endl << "Creating Octomap with updated freespace"
			<< std::endl;
	ros::Time start = ros::Time::now();

	newTree->clear();
	newTree->setResolution(tree.getResolution());
	// octomap::OcTree *newTree = &freeTree;
	newTree->setProbHit(1.0);
	newTree->setProbMiss(0.0);

	double x, y, z;
	tree.getMetricMin(x, y, z);
	octomap::point3d min(x, y, z);
	octomap::OcTreeKey kMin = tree.coordToKey(min);

	tree.getMetricMax(x, y, z);
	octomap::point3d max(x, y, z);
	octomap::OcTreeKey kMax = tree.coordToKey(max);

	// bool* free = new bool[kMax.k[0] - kMin.k[0]+1][kMax.k[1]-kMin.k[1]+1][kMax.k[2]-kMin.k[2]+1];
	int maxX = kMax.k[0] - kMin.k[0];
	int maxY = kMax.k[1] - kMin.k[1];
	int maxZ = kMax.k[2] - kMin.k[2];

	int kMaxX = kMax.k[0];
	int kMaxY = kMax.k[1];
	int kMaxZ = kMax.k[2];

	int kMinX = kMin.k[0];
	int kMinY = kMin.k[1];
	int kMinZ = kMin.k[2];

	std::vector<std::vector<std::vector<bool> > > occ;
	initOctomapArray(tree, occ, false);

	for (int x = kMin.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;
				octomap::OcTreeNode *node = tree.search(nk);

				if (node) {
					if (tree.isNodeOccupied(node)) {
						newTree->updateNode(nk, true, true);
						// newTree->updateNode(nk, true, true);
					}
				} else {
					// newTree->updateNode(nk, false, true);
				}

				if (!node) {
					// TODO: not performant...
					// TODO: maybe change 5 back to 4...
					for (int nx = std::max(x - distXZ, kMinX);
							nx <= std::min(x + distXZ, kMaxX); ++nx) {
						for (int ny = std::max(y - distY, kMinY);
								ny <= std::min(y + distY, kMaxY); ++ny) {
							for (int nz = std::max(z - distXZ, kMinZ);
									nz <= std::min(z + distXZ, kMaxZ); ++nz) {
								occ[nx - kMinX][ny - kMinY][nz - kMinZ] = true;
							}
						}
					}
				} else if (tree.isNodeOccupied(node)) {
					// If the node is occupied the XZ distance is also used for the Y direction.
					for (int nx = std::max(x - distXZ, kMinX);
							nx <= std::min(x + distXZ, kMaxX); ++nx) {
						for (int ny = std::max(y - distYObstacle, kMinY);
								ny <= std::min(y + distYObstacle, kMaxY);
								++ny) {
							for (int nz = std::max(z - distXZ, kMinZ);
									nz <= std::min(z + distXZ, kMaxZ); ++nz) {
								occ[nx - kMinX][ny - kMinY][nz - kMinZ] = true;
							}
						}
					}
				}

			}
		}
	}

	for (int x = kMin.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;
				octomap::OcTreeNode *node = tree.search(nk);

				if (node && !tree.isNodeOccupied(node)
						&& !occ[x - kMin.k[0]][y - kMin.k[1]][z - kMin.k[2]]) {
					newTree->updateNode(nk, false, true);
				}
			}
		}
	}

	// octomap::OcTreeNode *nodeOld = tree.search(kMin);
	octomap::OcTreeNode *nodeNew = newTree->search(kMin);
	if (!nodeNew) {
		newTree->updateNode(kMin, true, true);
	}
	// nodeOld = tree.search(kMax);
	nodeNew = newTree->search(kMax);
	if (!nodeNew) {
		newTree->updateNode(kMax, true, true);
	}

	newTree->updateInnerOccupancy();

	octomap::OcTreeKey ownKey = newTree->coordToKey(
			interestingPoints[interestingPoints.size() - 1].getOwnPosition(
					this));
	paintFree(*newTree, ownKey, 2, 1, 2);

	std::cout << "Creating new map took " << (ros::Time::now() - start).toSec()
			<< " seconds" << std::endl;

	saveOctomap(*newTree, "reallyFree.bt", distY * 10 + distXZ);
}

void LSDWrapper::visualizeMarkedPoints(octomap::OcTree* saveTo,
		std::vector<std::vector<std::vector<int> > >& marked,
		octomap::OcTree &originalTree) {
	traverseOctomap(originalTree,
			boost::bind(&LSDWrapper::visualizeMarkedPointsCallback, this, _1,
					_2, _3, _4, _5, saveTo, &marked));
	saveTo->updateInnerOccupancy();
}

void LSDWrapper::doOctomap(std_msgs::Empty msg) {

	if (!camDataSet) {
		ROS_WARN("Camera Data has not yet been set! Ignoring...");
		return;
	}

	callNumber++;

	if (callNumber == 1) {
		std::vector<DroneCommand> commands;
		generateLookAroundCommands(commands, 30,
				convertCameraToDroneCenter(
						lastCamToWorldWithoutTransform)/* eigenVecToOcto(lastCamToWorldWithoutTransform.translation())*/);
		addCommandsToQueue(commands);
		currControl = LOOK_AROUND;
		return;
	}

	std::cout << "first " << keyframes[1]->getCamToWorld().translation().x()
			<< " " << keyframes[1]->getCamToWorld().translation().y() << " "
			<< keyframes[1]->getCamToWorld().translation().z() << " "
			<< std::endl;
	std::cout << "other " << keyframes[4]->getCamToWorld().translation().x()
			<< " " << keyframes[4]->getCamToWorld().translation().y() << " "
			<< keyframes[4]->getCamToWorld().translation().z() << " "
			<< std::endl;

	bool shallReturn = false;
	if(wentToNewIntPoint)
	{
		ROS_INFO("Went to new point, doing Look-Around first.");
		std::vector<DroneCommand> commands;
		generateLookAroundCommands(commands, 30,
				convertCameraToDroneCenter(
						lastCamToWorldWithoutTransform), lastYaw/* eigenVecToOcto(lastCamToWorldWithoutTransform.translation())*/);
		addCommandsToQueue(commands);
		currControl = LOOK_AROUND;
		shallReturn = true;
	}

	if (interestingPoints.size() == 0 || wentToNewIntPoint) {
		ROS_INFO("Adding new current position as new interesting point.");
		wentToNewIntPoint = false;
		interestingPoints.push_back(
				InterestingPoint(lastCamToParent, lastParentToWorld,
						lastParentId));
	}

	if(shallReturn)
	{
		return;
	}

	bool landed = false;
	if(callNumber > 2)
	{
		// Octomap Creation will probably take very long -> Land meanwhile)
		ROS_INFO("Octomap creationg will probably take very long -> landing meanwhile to save battery life.");
		publish("c land");
		landed = true;
	}

	ROS_INFO("Creating Octomap");

	tree.setOccupancyThres(0.86);
	// tree.setOccupancyThres(0.7);
	// tree.setProbHit( 0.55);
	// tree.setProbMiss(0.4 );
	// tree.setProbHit(0.4);
	// tree.setProbMiss(0.8);
	std::cout << "Octomap values " << tree.getProbHit() << " "
			<< tree.getProbMiss() << " " << tree.getOccupancyThres() << " "
			<< tree.getClampingThresMax() << " " << tree.getClampingThresMin()
			<< std::endl;

	// tree.coordToKey(tree.getBBXCenter()).

	// tree.setOccupancyThres(0.5);
	ros::Time octomapStart = ros::Time::now();

	double scale = ptamWrapper->getScale();
	double resolution = 0.1 / scale;

	tree.clear();
	tree.setResolution(resolution);

	TimeMeasuring tOcto("Generating Octomap");



	/*double occupancyThres = 0.5;
	 for(int i = 0; i < 25; ++i)
	 {
	 tree.clear();
	 tree.setOccupancyThres(occupancyThres);

	 generateOctomap();

	 std::stringstream s;
	 s << "trees/thres" << occupancyThres << ".bt";
	 tree.writeBinary(s.str());
	 occupancyThres += 0.02;
	 }*/

	/*for(int i = 0; i < 5; ++i)
	 {
	 for(int j = i; j < 10; ++j)
	 {
	 tree.clear();

	 int sparse1 = i+1;
	 int sparse2 = j+1;

	 generateOctomap(sparse1, sparse2);

	 std::stringstream s;
	 s << "trees/spars" << sparse1 << "," << sparse2 << ".bt";
	 tree.writeBinary(s.str());

	 tree.clear();
	 }
	 }*/

	generateOctomap(2, 5);

	tOcto.end();

	ros::Duration octomapTime = ros::Time::now() - octomapStart;
	std::cout << "Generating the octomap took: " << octomapTime.toSec()
			<< " seconds" << std::endl;

	// ros::spinOnce();

	// TODO: change!!!!1------------------------------------------------------------------------------------------------------------------------------------------------
	/*tree.clear();
	 tree.readBinary("testTreeLoad.bt");*/

	double x, y, z;
	tree.getMetricMin(x, y, z);
	octomap::point3d min(x, y, z);
	//std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
	tree.getMetricMax(x, y, z);
	octomap::point3d max(x, y, z);
	//std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

	bool unknownAsOccupied = true;
	unknownAsOccupied = false;
	float maxDist = 1.0;
	//- the first argument ist the max distance at which distance computations are clamped
	//- the second argument is the octomap
	//- arguments 3 and 4 can be used to restrict the distance map to a subarea
	//- argument 5 defines whether unknown space is treated as occupied or free
	//The constructor copies data but does not yet compute the distance map
	/*DynamicEDTOctomap distmap(maxDist, &tree, min, max, unknownAsOccupied);
	 distmap.update();*/

	//std::cout << "distance " << distmap.getDistance(eigenVecToOcto(keyframes[10]->camToWorld.translation())) << std::endl;
	std::cout << "360... " << std::endl;

	// octomap::point3d startPoint = eigenVecToOcto(keyframes[4]->camToWorld.translation());
	octomap::point3d oldStartPoint = eigenVecToOcto(
			keyframes[4]->getCamToWorld().translation());

	std::cout << "old " << oldStartPoint << std::endl;

	// TODO (lsd): Maybe there are some problems with concurrency here... (not synchronized yet)
	octomap::point3d startPoint = eigenVecToOcto(
			lastCamToWorldWithoutTransform.translation());

	std::cout << "new " << startPoint << std::endl;

	/*double dist[360];

	 const int minifier = 30;
	 const int maxSmall = 359 / minifier + 1;
	 double angleSmall[maxSmall];
	 for (int i = 0; i < maxSmall; ++i) {
	 angleSmall[i] = 10000;
	 }
	 for (int angle = 0; angle < 360; ++angle) {
	 double x = sin(angle * M_PI / 180.0);
	 double y = cos(angle * M_PI / 180.0);

	 octomap::point3d direction(x, 0, y);
	 octomap::point3d endPoint;
	 tree.castRay(startPoint, direction, endPoint, false,
	 100);
	 double distance = endPoint.distance(startPoint);

	 double ySave = startPoint.y();
	 startPoint.y() = ySave + resolution;
	 tree.castRay(startPoint, direction, endPoint, true);
	 double dist2 = endPoint.distance(startPoint);

	 startPoint.y() = ySave - resolution;
	 tree.castRay(startPoint, direction, endPoint, true);
	 double dist3 = endPoint.distance(startPoint);

	 distance = std::min(distance, dist2);
	 distance = std::min(distance, dist3);

	 startPoint.y() = ySave;

	 // std::cout << distance << " " << dist2 << " " << dist3 << " " << angle << std::endl;

	 dist[angle] = distance;

	 int smallIndex = angle / minifier;
	 if (distance < angleSmall[smallIndex]) {
	 angleSmall[smallIndex] = distance;
	 }

	 }

	 std::cout << " start: " << convertToAutopilot(startPoint);

	 std::stringstream commands;
	 commands << "setStayTime 1" << std::endl;
	 double lastDist = -1;
	 octomap::point3d lastDirection;
	 double lastAngle;

	 double angles[maxSmall];
	 octomap::point3d ends[maxSmall];

	 for (int i = 0; i < maxSmall; ++i) {
	 double angle = i * minifier + minifier / 2.0;
	 double x = sin(angle * M_PI / 180.0);
	 double y = cos(angle * M_PI / 180.0);

	 angles[i] = angle;
	 // TODO (lsd): Add position of keyframe (not assuming and take care of angle...)s

	 // TODO (lsd): Make direction a little bit smaller...
	 double dist = angleSmall[i];
	 dist -= 0.5 / scale;
	 if (dist < 0) {
	 dist = 0;
	 }
	 octomap::point3d direction(x, 0, y);
	 octomap::point3d end = startPoint + direction * dist;

	 ends[i] = end;

	 if (lastDist > -1) {
	 if (lastDist > dist) {
	 // if last distance was bigger, move drone back a bit...
	 octomap::point3d myEnd = startPoint + lastDirection * dist;
	 addMoveCommand(commands, lastAngle, myEnd);
	 } else {
	 octomap::point3d myEnd = startPoint + direction * lastDist;
	 addMoveCommand(commands, angle, myEnd);
	 }
	 }

	 addMoveCommand(commands, angle, end);

	 lastDist = dist;
	 lastDirection = direction;
	 lastAngle = angle;
	 }
	 // std::cout << "Test commands: " << std::endl << commands.str() << std::endl;

	 std::stringstream newCommands;
	 for (int i = 0; i < maxSmall; ++i) {
	 double theAngle = angles[i] + 90;
	 if (theAngle > 360) {
	 theAngle -= 360;
	 }
	 addMoveCommand(newCommands, theAngle, ends[i]);
	 int otherIndex = (i + maxSmall / 2) % maxSmall;
	 // std::cout << "indices " << i << " " << otherIndex << std::endl;
	 addMoveCommand(newCommands, theAngle, ends[otherIndex]);
	 }

	 std::fstream file;
	 file.open("Commands.txt", std::fstream::out | std::fstream::trunc);
	 file << newCommands.str();
	 file.close();*/

	// for(int factor = 0; factor <= 15; ++factor){
	paintFree(tree, tree.coordToKey(interestingPoints[0].getOwnPosition(this)),
			4, 3, 4);

	tree.updateInnerOccupancy();

	convertToFreeTree(tree, "testTreeUnknown.bt");

	TimeMeasuring tUp("Updated Octomap (5,1)");
	createUpdatedOctomap(5, 1, &freeTree);
	tUp.end();

	TimeMeasuring tUp2("UpdatedOctomap(3,1)");
	createUpdatedOctomap(3, 1, &freeTreeBigger);
	tUp2.end();

	convertToFreeTree(freeTreeBigger, "unknownTree31.bt");

	std::vector<std::vector<DroneCommand> > starDiscoveries;
	starDiscoveries.resize(interestingPoints.size());

	std::vector<std::vector<std::vector<int> > > markedFromOne;
	initOctomapArray(tree, markedFromOne, 0);

	std::vector<std::vector<std::vector<int> > > markedFromOneSmaller;
	initOctomapArray(tree, markedFromOneSmaller, 0);

	int currPoint = interestingPoints.size() - 1;

	for (int i = 0; i < interestingPoints.size(); ++i) {

		InterestingPoint p = interestingPoints[i];

		std::vector<std::vector<std::vector<int> > > marked;

		MarkingArray markedBigger;

		initOctomapArray(freeTree, marked, 0);

		initOctomapArray(tree, markedBigger, 0);

		ros::Time t = ros::Time::now();

		// ros::spinOnce();

		TimeMeasuring tMark1("Mark points in sight 1");
		// p.markPointsInSight(&marked, 1, &freeTreeBigger, this, 5, true);
		p.markPointsInSight(&marked, 1, &freeTreeBigger, this, 5, 5);
		tMark1.end();

		ros::Duration t2 = ros::Time::now() - t;
		// std::cout << "Calculating points in line of sight took: " << t2.toSec() << " seconds" << std::endl;
		// ros::spinOnce();

		t = ros::Time::now();

		TimeMeasuring tMark2("Mark points in sight bigger");
		p.markPointsInSight(&markedBigger, 1, &tree, this, 0);
		tMark2.end();

		t2 = ros::Time::now() - t;
		// std::cout << "Calculating points in line of sight took: " << t2.toSec() << " seconds" << std::endl;

		// ros::spinOnce();

		octomap::OcTree *testTree = new octomap::OcTree(
				freeTree.getResolution());
		visualizeMarkedPoints(testTree, marked, freeTree);
		saveOctomap(*testTree, "InLineOfSight.bt", i);

		octomap::OcTree *testTreeBigger = new octomap::OcTree(
				tree.getResolution());
		visualizeMarkedPoints(testTreeBigger, markedBigger, tree);
		saveOctomap(*testTreeBigger, "InLineOfSightBigger.bt", i);

		std::vector<DroneCommand> alternativeStarDiscover;

		// TODO (lsd): Insert correct yaw...
		double size = p.computeNextStarDiscovery(&marked, 0, 0, this, &freeTree,
		/*&starDiscoveries[i]*/&alternativeStarDiscover);

		TimeMeasuring tStar("Compute Star Discovery");
		double size2 = p.computeNextStarDiscoveryOtherTactic(&marked, 0,
				lastYaw, this, &freeTreeBigger,
				&starDiscoveries[i] /*&alternativeStarDiscover*/);

		tStar.end();

		std::cout << "Sizes of Star Discoveries: " << size << " and " << size2
				<< std::endl;

		/*if(size <= 0.01)
		 {
		 starDiscoveries[i].clear();
		 p.computeNextStarDiscovery(&marked, 0, 0, this, &freeTreeBigger, &starDiscoveries[i]);
		 }*/

		octomap::OcTree *discoveryTree = new octomap::OcTree(
				tree.getResolution());
		visualizeCommands(freeTree, *discoveryTree, starDiscoveries[i]);
		saveOctomap(*discoveryTree, "Discovery.bt", i);
		delete discoveryTree;

		/*discoveryTree = new octomap::OcTree(tree.getResolution());
		visualizeCommands(freeTree, *discoveryTree, alternativeStarDiscover);
		saveOctomap(*discoveryTree, "AlternativeDiscovery.bt", i);
		delete discoveryTree;*/

		printCommands(starDiscoveries[i], "commands.txt");
		// printCommands(alternativeStarDiscover, "commandsAlternative.txt");

		// TODO (lsd): Choose best star discovery...

		TimeMeasuring tMarking("Filling array markedFromOne");

		for (int x = 0; x < markedBigger.size(); x++) {
			for (int y = 0; y < markedBigger[x].size(); ++y) {
				for (int z = 0; z < markedBigger[x][y].size(); ++z) {
					if (markedBigger[x][y][z] == 1) {
						markedFromOne[x][y][z] = 1;
					}
					if (marked[x][y][z] == 1) {
						markedFromOneSmaller[x][y][z] = 1;
					}
				}
			}
		}

		tMarking.end();

		delete testTree;
		delete testTreeBigger;

	}

	std::vector<std::vector<std::vector<int> > > possiblyInteresting,
			possiblyInterestingBigger;
	initOctomapArray(freeTree, possiblyInteresting, 0);
	initOctomapArray(freeTree, possiblyInterestingBigger, 0);
	// find free points, that are not in any Line Of Sight...

	TimeMeasuring tMark("Mark interesting points.");
	// TODO: maybe change back to freeTree
	traverseOctomap(freeTreeBigger,
			boost::bind(&LSDWrapper::markInterestingPointsCallback, this, _1,
					_2, _3, _4, _5, &possiblyInterestingBigger,
					&markedFromOne));

	traverseOctomap(freeTreeBigger,
			boost::bind(&LSDWrapper::markInterestingPointsCallback, this, _1,
					_2, _3, _4, _5, &possiblyInteresting,
					&markedFromOneSmaller)); // TODO: Change back or change name...

	tMark.end();

	/*traverseOctomap(freeTree,
	 boost::bind(&LSDWrapper::markInterestingPointsCallback, this, _1,
	 _2, _3, _4, _5, &possiblyInterestingBigger,
	 &markedFromOne));*/

	octomap::OcTree *testTree = new octomap::OcTree(freeTree.getResolution());
	visualizeMarkedPoints(testTree, possiblyInteresting, freeTreeBigger);
	saveOctomap(*testTree, "possiblyInteresting.bt");
	delete testTree;

	testTree = new octomap::OcTree(freeTree.getResolution());
	visualizeMarkedPoints(testTree, possiblyInterestingBigger, freeTreeBigger);
	saveOctomap(*testTree, "possiblyInterestingBigger.bt");
	delete testTree;

	TimeMeasuring tInteresting("Computing biggest area of interesting points");

	// Then do a little floodFill from all of those points...  maybe insert them into a vector also, so that not all points are traversed (unnecessary).
	// Maybe do a "rectangular floodfill" instead, to find the center point... (do this later, first a normal floodfill to keep it simple!)
	MarkingArray flooded;
	initOctomapArray(freeTree, flooded, 0);
	int currMarker = 1;
	for (int x = 0; x < possiblyInteresting.size(); x++) {
		for (int y = 0; y < possiblyInteresting[x].size(); ++y) {
			for (int z = 0; z < possiblyInteresting[x][y].size(); ++z) {
				if (possiblyInteresting[x][y][z] == 1
						&& flooded[x][y][z] == 0) {
					if (floodFill(x, y, z, possiblyInteresting, flooded,
							currMarker)) {
						currMarker++;
					}
				}
			}
		}
	}

	std::vector<int> numMarked;
	// 0-th element is not used... Not the best style, but it simplifies the following code a little bit...
	// TODO (lsd): actually resize(currMarker) should be enough...
	numMarked.resize(currMarker + 1);

	for (int x = 0; x < possiblyInteresting.size(); x++) {
		for (int y = 0; y < possiblyInteresting[x].size(); ++y) {
			for (int z = 0; z < possiblyInteresting[x][y].size(); ++z) {
				int c = flooded[x][y][z];
				if (c != 0) {
					numMarked[c]++;
				}
			}
		}
	}

	std::cout << std::endl << "There are " << currMarker - 1
			<< " different areas of possibly interesting points." << std::endl;

	int bestNumMarked = 0;
	int bestIndex = 0;
	for (int i = 0; i < numMarked.size(); ++i) {
		if (numMarked[i] > bestNumMarked) {
			bestNumMarked = numMarked[i];
			bestIndex = i;
		}
	}

	std::cout << "The best one has " << bestNumMarked << " voxels."
			<< std::endl;

	tInteresting.end();

	saveOctomap(tree, "tree.bt");

	if(landed)
	{
		ROS_INFO("Taking off again.");

		ros::Duration dur(0.5);

		while(ptamWrapper->lastNavinfoReceived.state == 2 || ptamWrapper->lastNavinfoReceived.state == 6 || ptamWrapper->lastNavinfoReceived.state == 0)
		{
			if(ptamWrapper->lastNavinfoReceived.state == 2 || ptamWrapper->lastNavinfoReceived.state == 0)
			{
				publish("c newtakeoff");
				publish("c start");
			}

			dur.sleep();

		}


		ROS_INFO("Taken off again successfully");
		/*publish("c newtakeoff");
		publish("c start");*/

	}



	bool goToNextPoint = false;

	// Take point at the middle. An alternative would be to take one (or more) random points.
	// TODO (lsd): Maybe just take a point, that is also in the smaller updatedOctomap...
	if (bestIndex != 0) {
		// int vertexInSmallerFreeSpace = -1;
		int centralVertex = numMarked[bestIndex] / 2;
		int currNumber = 0;
		octomap::point3d centralPos;

		std::vector<octomap::OcTreeKey> possibleCenter;

		// Indicates, if there is at least one voxel that is free in the 5-1 map.
		bool freeVoxelsExist = false;

		TimeMeasuring bestCenterM("Compute best CenterPoint in region.");

		for (int x = 0; x < possiblyInteresting.size(); x++) {
			for (int y = 0; y < possiblyInteresting[x].size(); ++y) {
				for (int z = 0; z < possiblyInteresting[x][y].size(); ++z) {
					int c = flooded[x][y][z];
					if (c == bestIndex) {

						octomap::OcTreeKey minKey = getOctomapMinKey(freeTree);
						octomap::OcTreeKey key;
						key[0] = x + minKey[0];
						key[1] = y + minKey[1];
						key[2] = z + minKey[2];

						octomap::OcTreeNode *node = freeTree.search(key);
						if (node && !freeTree.isNodeOccupied(node)) {
							if (!freeVoxelsExist) {
								freeVoxelsExist = true;
								possibleCenter.clear();
							}
							possibleCenter.push_back(key);
						} else if (!freeVoxelsExist) {
							possibleCenter.push_back(key);
						}

						// centralPos = freeTree.keyToCoord(key);

					}
				}
			}

		}

		// Look how big the Star Discovery will be for some random points.
		int numSamples = 200;
		if (possibleCenter.size() > numSamples) {
			std::random_shuffle(possibleCenter.begin(), possibleCenter.end());
		}
		int bestSize = 0;
		int bestCenter = -1;
		int until = std::min((int) possibleCenter.size(), numSamples);
		for (int i = 0; i < until; ++i) {
			octomap::OcTreeKey key = possibleCenter[i];
			std::vector<DroneCommand> commands;
			double size = InterestingPoint::computeNextStarDiscoveryOtherTactic(
					0, 0, 0, this, &freeTreeBigger, &commands,
					freeTree.keyToCoord(key));
			if (size >= bestSize) {
				bestSize = size;
				bestCenter = i;
			}
		}
		if (freeVoxelsExist) {
			std::cout << "Chose new center point in 5-1-map" << std::endl;
		} else {
			std::cout << "Chose new center in in 3-1-map" << std::endl;
		}
		centralPos = freeTree.keyToCoord(possibleCenter[bestCenter]);
		bestCenterM.end();

		/*MarkingArray nextMarked;
		initOctomapArray(freeTree, nextMarked, 0);
		InterestingPoint::markPointsInSight(&nextMarked, 1, &freeTree, this,
				centralPos, 0);
		std::vector<DroneCommand> discCommands;
		InterestingPoint::computeNextStarDiscovery(&nextMarked, 1, 0, this,
				&freeTree, &discCommands, centralPos);*/

		std::cout << std::endl << "The point chosen is at " << centralPos.x()
				<< ", " << centralPos.y() << ", " << centralPos.z()
				<< std::endl;

		std::vector<DroneCommand> way;

		TimeMeasuring tFindWay("Finding way to interesting point");
		octomap::point3d currPos = eigenVecToOcto(
				lastCamToWorldWithoutTransform.translation());

		bool betterWayFound = generatePathMoveCommands(currPos, centralPos, way, freeTree);

		bool wayFound = betterWayFound;
		if(!betterWayFound)
		{
			way.clear();
			wayFound = generatePathMoveCommands(currPos, centralPos, way,
							freeTreeBigger);
		}

		tFindWay.end();

		if(betterWayFound)
		{
			std::cout << "Found a way even in the 5,1-map!" << std::endl;
		}
		if (!wayFound) {
			std::cout << "Didn't find a way to the interesting point."
					<< std::endl;
		} else {
			std::cout << "Saving way to interesting point." << std::endl;
			octomap::OcTree visTree(freeTree.getResolution());
			visualizeCommands(freeTree, visTree, way);
			saveOctomap(visTree, "WayToInteresting.bt", 0);

			// TODO (lsd): Update...
			// if (interestingPoints[currPoint].didDiscover) {
			if (interestingPoints[currPoint].numStarDiscoveries >= 1) {
				addCommandsToQueue(way);
				goToNextPoint = true;
				currControl = PATH;
				wentToNewIntPoint = true;
				std::cout << "going to the interesting point!" << std::endl;
			}
		}

	}

	if (!goToNextPoint) {
		addCommandsToQueue(starDiscoveries[currPoint]);
		currControl = STAR_DISCOVERY;
		interestingPoints[currPoint].didDiscover = true;
		interestingPoints[currPoint].numStarDiscoveries++;
	}

	/*std::cout << "Call number: " << callNumber << ", factor: " << factor << std::endl;

	 callNumber++;
	 }*/

	// ros::spinOnce();
	// std::cout << "end 360" << std::endl;

	octomap::point3d direction(0.0, 0.0, 1.0);
	testRayCast(octomap::point3d(1, 0, 0), true);
	testRayCast(octomap::point3d(0, 1, 0), true);
	testRayCast(octomap::point3d(0, 0, 1), true);

	testRayCast(octomap::point3d(1, 0, 0), false);
	testRayCast(octomap::point3d(0, 1, 0), false);
	testRayCast(octomap::point3d(0, 0, 1), false);

	testRayCast(octomap::point3d(-1, 0, 0), true);
	testRayCast(octomap::point3d(0, -1, 0), true);
	testRayCast(octomap::point3d(0, 0, -1), true);

	testRayCast(octomap::point3d(-1, 0, 0), false);
	testRayCast(octomap::point3d(0, -1, 0), false);
	testRayCast(octomap::point3d(0, 0, -1), false);
	/*std::cout << "TestStartpoint:" << keyframes[5]->camToWorld.translation() << std::endl;
	 std::cout << "TestEndpoint: " << endPoint << std::endl;*/
	// std::cout << "TestEndpoint: " << endPoint.x() << " " << endPoint.y() << " " << endPoint.z() << std:.endl;
	Eigen::Vector3f p;
	// currentPositionOffset * p;

// 	distmap.getDistance();

/*	octomap::ColorOcTree colTree(tree.getResolution());
	colTree.updateNode(0, 0, 0, true, false);
	colTree.updateNode(1, 0, 0, true, false);
	colTree.updateNode(1, 1, 0, true, false);
	colTree.updateNode(0, 1, 0, true, false);
	colTree.setNodeColor(0, 0, 0, 255, 0, 0);
	colTree.setNodeColor(1, 0, 0, 0, 255, 0);
	colTree.setNodeColor(0, 1, 0, 0, 0, 255);
	colTree.setNodeColor(1, 1, 0, 255, 255, 0);
	colTree.writeBinary("colorTree.bt");*/

	// tree.writeBinary("testTree.bt");
	// ROS_INFO("Octomap saved to testTree.bt");

}

LSDKeyframe::LSDKeyframe() :
		id(0), camToWorld(), points(), poses(), inserted(false), camToWorldSet(
				false), constraints() {
}

std::vector<Sophus::Sim3f>* LSDKeyframe::getPoses() {
	return &poses;
}

void LSDKeyframe::insertedToOctomap() {
	poseInsertedToOctomap = camToWorld;
	inserted = true;
}

double LSDKeyframe::getDistanceToLastInserted() {
	if (inserted) {
		return (poseInsertedToOctomap.translation() - camToWorld.translation()).norm();
	}
	return 0.0;
}

Eigen::Vector3f LSDKeyframe::getVectorToLastInserted() {
	return (poseInsertedToOctomap.translation() - camToWorld.translation());
}

bool LSDKeyframe::wasInserted() {
	return inserted;
}

InterestingPoint::InterestingPoint(Sophus::Sim3f camToParent,
		Sophus::Sim3f firstParentToWorld, int keyframeId) :
		camToParent(camToParent), keyframeId(keyframeId), didDiscover(false), sumDiscoverDist(
				0.0), firstParentToWorld(firstParentToWorld), numStarDiscoveries(
				0) {
}

octomap::point3d makeSafer(octomap::point3d pos, octomap::point3d point) {
	// make the flight a bit safer...
	double dist = point.distance(pos);
	double newDist = dist;
	if (newDist > 0.6) {
		newDist -= 0.3;
	} else {
		newDist /= 2.0;
	}
	// make the flight a bit safer...
	octomap::point3d newPoint = (point - pos) * (newDist / dist) + pos;
	return newPoint;
}

double InterestingPoint::computeNextStarDiscovery(MarkingArray* marked,
		int markNumber, double currentYaw, LSDWrapper* wrapper,
		octomap::OcTree* tree, std::vector<DroneCommand>* commands,
		octomap::point3d pos) {

	// TODO (lsd): Change back to 30???
	const int minifier = 36;

	const int maxSmall = 359 / minifier + 1;
	double maxDist[maxSmall];
	octomap::point3d maxPoint[maxSmall];
	for (int i = 0; i < maxSmall; ++i) {
		maxDist[i] = 0;
		maxPoint[i] = pos;
	}

	octomap::OcTreeKey min = wrapper->getOctomapMinKey(*tree);

	MarkingArray &marked2 = *marked;
	for (int x = 0; x < marked2.size(); ++x) {
		for (int y = 0; y < marked2[x].size(); ++y) {
			for (int z = 0; z < marked2[x][y].size(); ++z) {
				if (marked2[x][y][z] == 1) {
					octomap::OcTreeKey key;
					key.k[0] = x + min.k[0];
					key.k[1] = y + min.k[1];
					key.k[2] = z + min.k[2];
					octomap::point3d p = tree->keyToCoord(key);
					double distance = p.distance(pos);
					octomap::point3d v = p - pos;
					if (v.x() != 0 || v.z() != 0) {
						double angle = getAngle(v);
						int smallAngle = angle / minifier;

						// std::cout << "test " << angle << " " << smallAngle << std::endl;
						if (distance > maxDist[smallAngle]) {
							maxDist[smallAngle] = distance;
							maxPoint[smallAngle] = p;
						}
					}
				}
			}
		}
	}

	int start = (currentYaw - 90);
	while (start < 0) {
		start += 360;
	}
	start = start / minifier;

	double res = 0.0;
	// int start = (currentYaw-90) / minifier;
	for (int num = 0; num < maxSmall; ++num) {
		int i = (start + num) % maxSmall;
		double angle = (i * minifier + minifier / 2.0) + 90;
		// double theAngle = angles[i]+90;

		if (angle > 360) {
			angle -= 360;
		}

		if (maxDist[i] > 0) {
			res += maxDist[i];
			// commands->push_back(DroneCommand(makeSafer(pos, maxPoint[i]),getAngle(maxPoint[i]) + 90));
			commands->push_back(
					DroneCommand(makeSafer(pos, maxPoint[i]), angle));
			// commands->push_back(DroneCommand(maxPoint[i], angle));
			// commands->push_back(DroneCommand(pos, getAngle(maxPoint[i]) + 90));
			commands->push_back(DroneCommand(pos, angle));
		}
		int otherIndex = (i + maxSmall / 2) % maxSmall;

		if (maxDist[otherIndex] > 0) {
			res += maxDist[otherIndex];
			// commands->push_back(DroneCommand(makeSafer(pos, maxPoint[otherIndex]),getAngle(maxPoint[otherIndex]) - 90));
			commands->push_back(
					DroneCommand(makeSafer(pos, maxPoint[otherIndex]), angle));
			// commands->push_back(DroneCommand(maxPoint[otherIndex], angle));
			// commands->push_back(DroneCommand(pos, getAngle(maxPoint[otherIndex]) - 90));
			commands->push_back(DroneCommand(pos, angle));
		}
	}

	commands->push_back(DroneCommand(pos, 0));
	return res;
}

void addWaypointSafe(octomap::point3d start, octomap::point3d end, double angle, std::vector<DroneCommand>* commands, double resolution)
{
	double scale = 0.1 / resolution;
	// In voxels...
	double safeDist = 1.4;

	double distance = end.distance(start) * scale;

	int num = (int) floor(distance / safeDist) + 1; // Wenn ein Zwischenpunkt, dann gleich 2

	for(int i = 0; i < num; ++i)
	{
		double endFactor = ((double) (i+1)/num);
		double startFactor = (1-endFactor);
		octomap::point3d pos = start * startFactor + end * endFactor;
		commands->push_back(DroneCommand(pos, angle));
	}

}

double angle2rad(double val)
{
	return val / 180.0 * M_PI;
}

double rad2angle(double val)
{
	double ret = val / M_PI * 180;
	while(ret < 0)
	{
		ret += 360;
	}
	return ret;
}

double InterestingPoint::computeNextStarDiscoveryOtherTactic(
		MarkingArray* marked, int markNumber, double currentYaw,
		LSDWrapper* wrapper, octomap::OcTree* tree,
		std::vector<DroneCommand>* commands, octomap::point3d pos) {

	const int minifier = 36;


	const int maxSmall = 359 / minifier + 1;
	double maxDist[maxSmall];
	octomap::point3d maxPoint[maxSmall];
	for (int i = 0; i < maxSmall; ++i) {
		maxDist[i] = 0;
		maxPoint[i] = pos;
	}

	octomap::OcTreeKey min = wrapper->getOctomapMinKey(*tree);

	// int start = (currentYaw - 90);
	int start = (0 - 90);
	while (start < 0) {
		start += 360;
	}
	start = start / minifier;

	while (currentYaw < 0) {
		currentYaw += 360;
	}
	/*while(currentYaw > 360)
	 {
	 currentYaw -= 360;
	 }*/

	// Added if the distance in one direction is zero.
	octomap::point3d maxAltPoint, minAltPoint;
	tree->castRay(pos, octomap::point3d(0, 1, 0), maxAltPoint, false, 100);
	tree->castRay(pos, octomap::point3d(0, -1, 0), minAltPoint, false, 100);
	octomap::point3d altPoint;
	if (maxAltPoint.distance(pos) >= minAltPoint.distance(pos)) {
		altPoint = maxAltPoint;
	} else {
		altPoint = minAltPoint;
	}
	double altDist = altPoint.distance(pos);
	// std::cout << "Alternative distance: " << altDist << std::endl;

	double resolution = tree->getResolution();
	// one Voxel equals 10 Centimeters.
	// TODO: Scale has to be used!
	for (int num = 0; num < maxSmall; ++num) {
		int i = (start + num) % maxSmall;
		double angle = (i * minifier + minifier / 2.0) + currentYaw;

		double realAngle = angle + 90;

		// double theAngle = angles[i]+90;

		while (angle > 360) {
			angle -= 360;
		}
		while (realAngle > 360) {
			realAngle -= 360;
		}

		double x = sin(angle * M_PI / 180.0);
		double y = cos(angle * M_PI / 180.0);

		octomap::point3d direction(x, 0, y);
		octomap::point3d endPoint;
		tree->castRay(pos, /*startPoint + */direction, endPoint, false, 100);
		double distance = endPoint.distance(pos);
		if (distance >= 4 * resolution) {
			// No making smaller until 40 cm.
			distance = std::max(resolution * 4, distance - 3 * resolution);
		}
		if (distance >= 8 * resolution) {
			// distance = std::max(8 * resolution, distance - 4 * resolution);
			distance = std::max(8 * resolution, distance - 3 * resolution);
		}
		/*if(distance > resolution * 2)
		 {
		 distance = std::max(resolution * 2, distance -  2 *resolution);
		 }
		 if(distance > 6 * resolution)
		 {
		 distance -= 3 * resolution;
		 }else
		 {
		 distance /= 2.0;
		 }*/

		octomap::point3d tar = pos + direction * distance;

		maxDist[i] = distance;
		maxPoint[i] = tar;
	}

	double res = 0.0;
	// int start = (currentYaw-90) / minifier;
	double lastAngle = currentYaw;
	for (int num = 0; num < maxSmall; ++num) {

		int i = (start + num) % maxSmall;
		double angle = (i * minifier + minifier / 2.0) + currentYaw + 90;

		while (angle >= 360) {
			angle -= 360;
		}
		// double realAngle = angle + 90;

		int otherIndex = (i + maxSmall / 2) % maxSmall;

		// Eigen::Quaterniond quat = Eigen::AngleAxisd()

		double blend = 0.5;
		double ix = sin(angle2rad(angle));
		double iy = cos(angle2rad(angle));
		double jx = sin(angle2rad(lastAngle));
		double jy = cos(angle2rad(lastAngle));

		double betweenAngle =  rad2angle(atan2(ix-(ix-jx)*blend,iy-(iy-jy)*blend));

		/*double added = angle + lastAngle;
		while(added < 0)
		{
			added += 360;
		}
		while(added >= 360)
		{
			added -= 360;
		}
		double betweenAngle = added / 2; */

		// std::cout << "Angle: " << angle << ", between angle " << betweenAngle << std::endl;

		// std::cout << "MaxDist... " << maxDist[i] << " other dist: " << maxDist[otherIndex] << std::endl;
		if (maxDist[i] < 2 * resolution && maxDist[otherIndex] < 2 * resolution
				&& altDist > maxDist[i] && altDist > maxDist[otherIndex]) {
			// std::cout << "Using altDist: " << altDist << std::endl;
			commands->push_back(DroneCommand(altPoint, betweenAngle));
			commands->push_back(DroneCommand(pos, angle));
		} else {
			bool angleChanged = false;
			if (maxDist[i] >= 1.8 * resolution) {
				res += maxDist[i];
				addWaypointSafe(pos, maxPoint[i], betweenAngle, commands, resolution);
				addWaypointSafe(maxPoint[i], pos, angle, commands, resolution);
				/*commands->push_back(DroneCommand(maxPoint[i], betweenAngle));
				commands->push_back(DroneCommand(pos, angle));*/
				angleChanged = true;
			}

			if (maxDist[otherIndex] >= 1.8 * resolution) {
				res += maxDist[otherIndex];
				addWaypointSafe(pos, maxPoint[otherIndex], angleChanged ? angle : betweenAngle, commands, resolution);
				addWaypointSafe(maxPoint[otherIndex], pos, angle, commands, resolution);
				/*commands->push_back(
						DroneCommand(maxPoint[otherIndex],
								angleChanged ? angle : betweenAngle));
				commands->push_back(DroneCommand(pos, angle));*/
			}
		}

		lastAngle = angle;

	}

	commands->push_back(DroneCommand(pos, currentYaw));

	commands->push_back(DroneCommand(altPoint, currentYaw));
	commands->push_back(DroneCommand(pos, currentYaw));

	return res;

}

double InterestingPoint::computeNextStarDiscovery(MarkingArray* marked,
		int markNumber, double currentYaw, LSDWrapper *wrapper,
		octomap::OcTree *tree, std::vector<DroneCommand>* commands) {
	octomap::point3d pos = this->getOwnPosition(wrapper);

	return computeNextStarDiscovery(marked, markNumber, currentYaw, wrapper,
			tree, commands, pos);
}

double InterestingPoint::computeNextStarDiscoveryOtherTactic(
		MarkingArray* marked, int markNumber, double currentYaw,
		LSDWrapper *wrapper, octomap::OcTree *tree,
		std::vector<DroneCommand>* commands) {
	octomap::point3d pos = this->getOwnPosition(wrapper);

	octomap::OcTreeKey k = tree->coordToKey(pos);
	k[1] -= 3;
	double bestSize;
	for (int heightDisp = -3; heightDisp <= 3; ++heightDisp) {
		octomap::point3d nextPos = tree->keyToCoord(k);
		std::vector<DroneCommand> coms;
		double size = computeNextStarDiscoveryOtherTactic(marked, markNumber,
				currentYaw, wrapper, tree, &coms, nextPos);

		if (heightDisp == 0) {
			size += 1; // Advantage for no height displacement.
		}

		if (size > bestSize) {
			bestSize = size;
			(*commands) = coms;
		}

		k[1]++;
	}
	return bestSize;
	// return computeNextStarDiscoveryOtherTactic(marked, markNumber, currentYaw, wrapper,tree, commands, pos);
}

namespace octomap {
typedef OcTreeNode NODE;

struct TripleCoords {
	TripleCoords(int x, int y, int z) :
			x(x), y(y), z(z) {
	}
	int x;
	int y;
	int z;
};

const int treeMaxVal = 32768;
// Note: the searched tree must be marked in the array before doing this...LSDWrapper
// This method should only be called with a free voxel as the starting point.
bool newCastRay(OcTree *octomap, const point3d& origin,
		const point3d& directionP, point3d& end, bool ignoreUnknown,
		double maxRange, MarkingArray& marked, int mark, int threshold) {

	/// ----------  see OcTreeBase::computeRayKeys  -----------

	// Initialization phase -------------------------------------------------------
	double x, y, z;
	octomap->getMetricMin(x, y, z);
	octomap::point3d min(x, y, z);
	octomap::OcTreeKey kMin = octomap->coordToKey(min);

	int minX = kMin.k[0];
	int minY = kMin.k[1];
	int minZ = kMin.k[2];

	// std::vector<TripleCoords> toMark;
	// int markWith = 0;

	OcTreeKey current_key;
	if (!octomap->coordToKeyChecked(origin, current_key)) {
		OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
		return false;
	}

	NODE* startingNode = octomap->search(current_key);
	if (startingNode) {
		if (octomap->isNodeOccupied(startingNode)) {
			// Occupied node found at origin
			// (need to convert from key, since origin does not need to be a voxel center)
			end = octomap->keyToCoord(current_key);
			return true;
		}
	} else if (!ignoreUnknown) {
		end = octomap->keyToCoord(current_key);
		return false;
	}

	point3d direction = directionP.normalized();
	bool max_range_set = (maxRange > 0.0);

	int step[3];
	double tMax[3];
	double tDelta[3];

	for (unsigned int i = 0; i < 3; ++i) {
		// compute step direction
		if (direction(i) > 0.0)
			step[i] = 1;
		else if (direction(i) < 0.0)
			step[i] = -1;
		else
			step[i] = 0;

		// compute tMax, tDelta
		if (step[i] != 0) {
			// corner point of voxel (in direction of ray)
			double voxelBorder = octomap->keyToCoord(current_key[i]);
			voxelBorder += double(step[i] * octomap->getResolution() * 0.5);

			tMax[i] = (voxelBorder - origin(i)) / direction(i);
			tDelta[i] = octomap->getResolution() / fabs(direction(i));
		} else {
			tMax[i] = std::numeric_limits<double>::max();
			tDelta[i] = std::numeric_limits<double>::max();
		}
	}

	if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
		OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
		return false;
	}

	// for speedup:
	double maxrange_sq = maxRange * maxRange;

	// Incremental phase  ---------------------------------------------------------

	int rx = current_key.k[0] - minX;
	int ry = current_key.k[1] - minY;
	int rz = current_key.k[2] - minZ;
	/*if(marked[rx][ry][rz] == mark || marked[rx][ry][rz] == occMark)
	 {
	 return false;
	 }else
	 {
	 // The start point shall also be marked later.
	 toMark.push_back(TripleCoords(rx, ry, rz));
	 }*/
	marked[rx][ry][rz] = mark;

	bool done = false;

	int numOut = 0;

	while (!done) {
		unsigned int dim;

		// find minimum tMax:
		if (tMax[0] < tMax[1]) {
			if (tMax[0] < tMax[2])
				dim = 0;
			else
				dim = 2;
		} else {
			if (tMax[1] < tMax[2])
				dim = 1;
			else
				dim = 2;
		}

		// check for overflow:
		if ((step[dim] < 0 && current_key[dim] == 0)
				|| (step[dim] > 0 && current_key[dim] == 2 * treeMaxVal - 1)) {
			OCTOMAP_WARNING(
					"Coordinate hit bounds in dim %d, aborting raycast\n", dim);
			// return border point nevertheless:
			end = octomap->keyToCoord(current_key);

			// not in line of sight.
			return false;
		}

		// advance in direction "dim"
		current_key[dim] += step[dim];
		tMax[dim] += tDelta[dim];

		// generate world coords from key
		end = octomap->keyToCoord(current_key);

		// check for maxrange:
		if (max_range_set) {
			double dist_from_origin_sq(0.0);
			for (unsigned int j = 0; j < 3; j++) {
				dist_from_origin_sq += ((end(j) - origin(j))
						* (end(j) - origin(j)));
			}
			if (dist_from_origin_sq > maxrange_sq)
				return false;

		}

		int rx = current_key.k[0] - minX;
		int ry = current_key.k[1] - minY;
		int rz = current_key.k[2] - minZ;
		/*if(marked[rx][ry][rz] == mark)
		 {
		 // When other voxel was already marked free, do so with the voxels, that were passed this raycast.
		 markWith = mark;
		 done = true;
		 break;
		 }else if(marked[rx][ry][rz] == occMark)
		 {
		 // When occupied, mark the passed voxels as occupied, too.
		 markWith = occMark;
		 done = true;
		 break;
		 }else
		 {
		 toMark.push_back(TripleCoords(rx, ry, rz));
		 }*/

		NODE* currentNode = octomap->search(current_key);
		if (currentNode) {
			if (octomap->isNodeOccupied(currentNode)) {
				// markWith = occMark;
				if (numOut < threshold) {
					numOut++;
					// marked[rx][ry][rz] = 1;
				} else {
					done = true;
					return false;
				}
			} else {
				numOut = 0;
				marked[rx][ry][rz] = 1;
			}
			// otherwise: node is free and valid, raycasting continues
		} else if (!ignoreUnknown) { // no node found, this usually means we are in "unknown" areas
			// markWith = occMark;
			if (numOut < threshold) {
				numOut++;
				// marked[rx][ry][rz] = 1;
			} else {
				done = true;
				return false;
			}
		}

	} // end while

	/*for(int i = 0; i < toMark.size(); ++i)
	 {
	 TripleCoords c = toMark[i];
	 marked[c.x][c.y][c.z] = markWith;
	 }*/

	return false;
}
}

void getNumVoxels(int x, int y, int z, octomap::OcTreeKey k, octomap::OcTree& t,
		int *numFree, int *numOcc) {
	// std::cout << "Test1" << std::endl;
	octomap::OcTreeNode *n = t.search(k);
	if (n) {
		// std::cout << "Blablabla" << std::endl;
		if (t.isNodeOccupied(n)) {
			(*numOcc)++;
		} else {
			(*numFree)++;
		}
	}
}

void LSDWrapper::analyseOctree(std::string filename) {

	octomap::OcTree t(0.1);
	t.readBinary(filename);

	octomap::OcTreeKey kMin = getOctomapMinKey(t);
	octomap::OcTreeKey kMax = getOctomapMaxKey(t);

	octomap::OcTreeKey boundingSize = kMax - kMin;

	std::cout << "analysing " << filename << std::endl;

	std::cout << "boundingBox size: " << boundingSize[0] << " x "
			<< boundingSize[1] << " x " << boundingSize[2] << std::endl;
	std::cout << "n^3 = " << boundingSize[0] * boundingSize[1] * boundingSize[2]
			<< std::endl;

	std::cout << "NumLeafs: " << t.getNumLeafNodes() << std::endl;
	std::cout << "Resolution: " << t.getResolution() << std::endl;

	int numFree = 0;
	int numOcc = 0;

	traverseOctomap(t,
			boost::bind(getNumVoxels, _1, _2, _3, _4, _5, &numFree, &numOcc));

	std::cout << "Num free voxels: " << numFree << std::endl;
	std::cout << "Num occ voxels: " << numOcc << std::endl;
	std::cout << "Number of total voxels: " << numFree + numOcc << std::endl;

	std::cout << "Unknown voxels in bounding box: "
			<< (boundingSize[0] * boundingSize[1] * boundingSize[2]) - numFree
					- numOcc << std::endl;

}

octomap::point3d InterestingPoint::getOwnPosition(LSDWrapper* wrapper) {
	Sophus::Sim3f keyCamToWorld = firstParentToWorld;
	LSDKeyframe* keyframe = wrapper->getKeyframeById(keyframeId);
	if (keyframe->isCamToWorldSet()) {
		keyCamToWorld = keyframe->getCamToWorld();
	}
	// octomap::point3d myPos = wrapper->eigenVecToOcto((keyCamToWorld * camToParent).translation());
	octomap::point3d myPos = wrapper->convertCameraToDroneCenter(
			keyCamToWorld * camToParent);
	return myPos;
}

void InterestingPoint::markPointsInSight(MarkingArray* marked, int markNumber,
		octomap::OcTree* tree, LSDWrapper* wrapper, octomap::point3d myPos,
		int threshold) {

	octomap::OcTreeNode *node = tree->search(myPos);
	if (!node || tree->isNodeOccupied(node)) {
		std::cout << "PROBLEM: Interesting point is actually occupied!!!";
	}
	octomap::OcTreeKey key = tree->coordToKey(myPos);

	octomap::OcTreeKey kMin = wrapper->getOctomapMinKey(*tree);

	octomap::OcTreeKey kMax = wrapper->getOctomapMaxKey(*tree);

	(*marked)[key.k[0] - kMin.k[0]][key.k[1] - kMin.k[1]][key.k[2] - kMin.k[2]] =
			1;

	int number = 0;

	octomap::OcTreeKey theoretical = kMax - kMin;
	std::cout << "Size of the map " << theoretical[0] << ' ' << theoretical[1]
			<< ' ' << theoretical[2] << std::endl;
	std::cout << "n^3 = " << theoretical[0] * theoretical[1] * theoretical[2];
	bool stop = false;
	for (int x = kMin.k[0]; x <= kMax.k[0] && !stop; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1] && !stop; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2] && !stop; ++z) {

				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;

				// only take the perimeter of the room...
				if (!(x == kMin[0] || x == kMax[0] || y == kMin[1]
						|| y == kMax[1] || z == kMin[2] || z == kMax[2])) {
					continue;
				}
				number++;

				octomap::point3d end;
				octomap::point3d dir = tree->keyToCoord(nk) - myPos;

				// bool newCastRay(OcTree *octomap, const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknown, double maxRange, MarkingArray& marked, int mark, int occMark) {
				newCastRay(tree, myPos, dir, end, false, 100.0, *marked,
						markNumber, threshold);

				/*octomap::KeyRay ray;
				 tree->computeRayKeys(myPos, tree->keyToCoord(nk), ray);
				 for(octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); ++it)
				 {
				 octomap::OcTreeKey k = *it;
				 octomap::OcTreeNode *node = tree->search(k);
				 if(!node || tree->isNodeOccupied(node))
				 {
				 break;
				 }

				 octomap::OcTreeKey arrayIndex = k - kMin;

				 (*marked)[arrayIndex[0]][arrayIndex[1]][arrayIndex[2]] = 1;

				 }*/
			}
		}
	}

	std::cout << "Test... " << number << std::endl;
}

void InterestingPoint::markPointsInSightOld(MarkingArray* marked,
		int markNumber, octomap::OcTree* tree, LSDWrapper* wrapper,
		octomap::point3d myPos) {

	octomap::OcTreeNode *node = tree->search(myPos);
	if (!node || tree->isNodeOccupied(node)) {
		std::cout << "PROBLEM: Interesting point is actually occupied!!!";
	}
	octomap::OcTreeKey key = tree->coordToKey(myPos);

	octomap::OcTreeKey kMin = wrapper->getOctomapMinKey(*tree);

	octomap::OcTreeKey kMax = wrapper->getOctomapMaxKey(*tree);

	(*marked)[key.k[0] - kMin.k[0]][key.k[1] - kMin.k[1]][key.k[2] - kMin.k[2]] =
			1;

	int number = 0;

	octomap::OcTreeKey theoretical = kMax - kMin;
	std::cout << "Size of the map " << theoretical[0] << ' ' << theoretical[1]
			<< ' ' << theoretical[2] << std::endl;
	std::cout << "n^3 = " << theoretical[0] * theoretical[1] * theoretical[2];

	for (int x = kMin.k[0]; x <= kMax.k[0]; ++x) {
		for (int y = kMin.k[1]; y <= kMax.k[1]; ++y) {
			for (int z = kMin.k[2]; z <= kMax.k[2]; ++z) {
				octomap::OcTreeKey nk;
				nk.k[0] = x;
				nk.k[1] = y;
				nk.k[2] = z;

				// only take the perimeter of the room...
				if (!(x == kMin[0] || x == kMax[0] || y == kMin[1]
						|| y == kMax[1] || z == kMin[2] || z == kMax[2])) {
					continue;
				}
				number++;

				octomap::KeyRay ray;
				tree->computeRayKeys(myPos, tree->keyToCoord(nk), ray);
				for (octomap::KeyRay::iterator it = ray.begin();
						it != ray.end(); ++it) {
					octomap::OcTreeKey k = *it;
					octomap::OcTreeNode *node = tree->search(k);
					if (!node || tree->isNodeOccupied(node)) {
						break;
					}

					octomap::OcTreeKey arrayIndex = k - kMin;

					// TODO (lsd): Put away (just for time measuring...
					// (*marked)[arrayIndex[0]][arrayIndex[1]][arrayIndex[2]] = 1;

				}
			}
		}
	}

	std::cout << "Test... " << number << std::endl;
}

void InterestingPoint::markPointsInSight(MarkingArray* marked, int markNumber,
		octomap::OcTree* tree, LSDWrapper *wrapper, int threshold, int hDisp) {

	TimeMeasuring t("Calculating points in sight new");

	octomap::point3d pos = getOwnPosition(wrapper);

	if (true) {
		octomap::OcTreeKey k = tree->coordToKey(pos);
		k[1] -= hDisp;
		double bestSize;
		for (int heightDisp = -hDisp; heightDisp <= hDisp; ++heightDisp) {
			octomap::point3d nextPos = tree->keyToCoord(k);

			markPointsInSight(marked, markNumber, tree, wrapper, nextPos,
					threshold);

			k[1]++;
		}
	} else {
		markPointsInSight(marked, markNumber, tree, wrapper, pos, threshold);
	}

	t.end();

	/*
	 TimeMeasuring t2("Calculate points in sight old");
	 markPointsInSightOld(marked, markNumber, tree, wrapper, myPos);
	 t2.end();*/
}

void swapCoordinates(int x, int y, int z, octomap::OcTreeKey key,
		octomap::OcTree &originalTree, octomap::OcTree *newTree) {
	octomap::OcTreeNode *node = originalTree.search(key);
	octomap::point3d pos = originalTree.keyToCoord(key);
	octomap::point3d newPos(pos.x(), pos.z(), -pos.y());
	if (node) {
		if (originalTree.isNodeOccupied(node)) {
			newTree->updateNode(newPos, true, true);
		} else {
			newTree->updateNode(newPos, false, true);
		}
	}
}

void LSDWrapper::saveOctomap(octomap::OcTree &octree, std::string name,
		int number) {

	std::stringstream stream;
	stream << "Flight 1/";
	stream << "Run " << callNumber << "/";

	std::string dir = stream.str();
	if (!boost::filesystem::exists(dir)) {
		boost::filesystem::create_directories(dir);
	}

	if (number != -1) {
		stream << number;
	}
	stream << name;

	octomap::OcTree transformedTree(octree.getResolution());

	// swap y and z coordinates, because the Octree-Viewer works better like this.
	traverseOctomap(octree,
			boost::bind(swapCoordinates, _1, _2, _3, _4, _5, &transformedTree));

	transformedTree.updateInnerOccupancy();

	transformedTree.writeBinary(stream.str());
	// octree.writeBinary(stream.str());

}

void LSDWrapper::printCommands(std::vector<DroneCommand>& commands,
		std::string name) {
	std::stringstream stream;
	stream << "Flight 1/";
	stream << "Run " << callNumber << "/";
	stream << name;

	std::fstream f;
	f.open(stream.str().c_str(), std::fstream::out | std::fstream::trunc);

	std::stringstream commandStream;

	for (int i = 0; i < commands.size(); ++i) {
		addMoveCommand(commandStream, commands[i].yaw, commands[i].position);
	}

	f << commandStream.str();

	f.close();

}

void LSDWrapper::visualizeMarkedPointsCallback(int x, int y, int z,
		octomap::OcTreeKey nk, octomap::OcTree &octomap,
		octomap::OcTree *newOctomap,
		std::vector<std::vector<std::vector<int> > > *marked) {
	octomap::OcTreeNode *node = octomap.search(nk);
	if (node != 0 && octomap.isNodeOccupied(node)) {
		newOctomap->updateNode(nk, true, true);
	} else if ((*marked)[x][y][z] == 1) {
		newOctomap->updateNode(nk, false, true);
	}

}

void LSDWrapper::markInterestingPointsCallback(int x, int y, int z,
		octomap::OcTreeKey nk, octomap::OcTree& octomap,
		MarkingArray* interesting, MarkingArray* uninteresting) {

	octomap::OcTreeNode *node = octomap.search(nk);
	if (node != 0 && !octomap.isNodeOccupied(node)
			&& (*uninteresting)[x][y][z] != 1) {
		(*interesting)[x][y][z] = 1;
	}

}

DroneCommand::DroneCommand(octomap::point3d position, double yaw) :
		position(position), yaw(yaw) {
}

void LSDWrapper::visualizeCommands(octomap::OcTree& tree,
		octomap::OcTree& newTree, std::vector<DroneCommand> &commands) {
	traverseOctomap(tree,
			boost::bind(&LSDWrapper::copyOccupiedPointsCallback, this, _1, _2,
					_3, _4, _5, &newTree));
	for (int i = 0; i < commands.size(); ++i) {
		octomap::point3d p = commands[i].position;
		newTree.updateNode(newTree.coordToKey(p), false, true);
	}

	newTree.updateInnerOccupancy();
}

// Returns true if something was marked.
bool LSDWrapper::floodFill(int x, int y, int z, MarkingArray& marked,
		MarkingArray& flooded, int currMarker) {

	if (x < 0 || y < 0 || z < 0 || x >= marked.size() || y >= marked[x].size()
			|| z >= marked[x][y].size()) {
		return false;
	}

	if (marked[x][y][z] != 1) {
		return false;
	}

	if (flooded[x][y][z] != 0) {
		return false;
	}

	flooded[x][y][z] = currMarker;
	floodFill(x + 1, y, z, marked, flooded, currMarker);
	floodFill(x, y + 1, z, marked, flooded, currMarker);
	floodFill(x, y, z + 1, marked, flooded, currMarker);
	floodFill(x - 1, y, z, marked, flooded, currMarker);
	floodFill(x, y - 1, z, marked, flooded, currMarker);
	floodFill(x, y, z - 1, marked, flooded, currMarker);

	return true;

}

void LSDWrapper::copyOccupiedPointsCallback(int x, int y, int z,
		octomap::OcTreeKey nk, octomap::OcTree& octomap,
		octomap::OcTree* newOctomap) {
	octomap::OcTreeNode *node = octomap.search(nk);
	if (node && octomap.isNodeOccupied(node)) {
		newOctomap->updateNode(nk, true, true);
	}
}

struct BFSStruct {
	octomap::OcTreeKey parent;
	double distance;

	BFSStruct() :
			distance(-1) {
	}

	BFSStruct(octomap::OcTreeKey parent, double distance) :
			parent(parent), distance(distance) {
	}

	bool operator<(const BFSStruct &o) const {
		return distance > o.distance;
	}

};

bool LSDWrapper::generatePathMoveCommands(octomap::point3d start,
		octomap::point3d end, std::vector<DroneCommand>& commands,
		octomap::OcTree &octomap) {

	octomap::OcTree &o = octomap;

	octomap::OcTreeKey kMin = getOctomapMinKey(o);

	octomap::OcTreeKey s = o.coordToKey(start) - kMin;
	octomap::OcTreeKey e = o.coordToKey(end) - kMin;

	if (s == e) {
		return true;
	}

	// In this queue the field parent is actually the current key.
	std::priority_queue<BFSStruct> next;

	std::vector<std::vector<std::vector<BFSStruct> > > array;
	initOctomapArray(o, array, BFSStruct());

	next.push(BFSStruct(e, 0));
	array[e[0]][e[1]][e[2]].distance = 0;

	bool finished = false;

	while (!next.empty() && !finished) {
		BFSStruct n = next.top();
		next.pop();
		octomap::OcTreeKey k = n.parent;

		if (k == s) {
			finished = true;
		}

		if (array[k[0]][k[1]][k[2]].distance < n.distance) {
			// outdated node, ignore
			continue;
		}

		//std::cout << "while " << k << std::endl;

		BFSStruct parent = array[k[0]][k[1]][k[2]];

		octomap::point3d distanceOrigin(k[0], k[1], k[2]);

		// Note: Maybe this should be changed, so that linear flight paths are preferred over diagonal flight paths (first process the voxels, where only one of the coordinates change, afterwards the other voxels.
		for (int x = k[0] - 1; x <= k[0] + 1 && !finished; ++x) {
			for (int y = k[1] - 1; y <= k[1] + 1 && !finished; ++y) {
				for (int z = k[2] - 1; z <= k[2] + 1 && !finished; ++z) {
					// Note, can also be the point equal k...

					if (x < 0 || y < 0 || z < 0 || x >= array.size()
							|| y >= array[x].size()
							|| z >= array[x][y].size()) {
						continue;
					}

					octomap::OcTreeKey kCurr(x, y, z);
					// std::cout << "curr: " << kCurr << std::endl;
					BFSStruct &curr = array[x][y][z];

					if (curr.distance != -2.0) {
						// If node is free.
						// compute distance adding.
						double distAdding = distanceOrigin.distance(
								octomap::point3d(x, y, z));
						double newDist = parent.distance + distAdding;

						if (curr.distance == -1.0) {
							// When visited the first time, check if free.

							// std::cout << "bla" << std::endl;
							// Node not processed yet.
							octomap::OcTreeKey kReal = kCurr + kMin;
							octomap::OcTreeNode *node = o.search(kReal);
							if (node == 0 || o.isNodeOccupied(node)) {
								// not free.
								curr.distance = -2;
								continue;
							}
						}
						if (curr.distance == -1 || newDist < curr.distance) {
							curr.distance = newDist;
							curr.parent = k;

							// std::cout << "push" << std::endl;
							next.push(BFSStruct(kCurr, newDist));
						}
					}
				}
			}
		}
	}

	std::cout << "start at: " << s << std::endl;

	if (!finished) {
		return false;
	}

	double angle = InterestingPoint::getAngle(end - start);

	octomap::OcTreeKey c = s;
	while (c != e) {
		c = array[c[0]][c[1]][c[2]].parent;

		// std::cout << "c: " << c << std::endl;

		octomap::point3d pos = o.keyToCoord(c + kMin);
		// std::cout << "pos: " << pos << std::endl;

		commands.push_back(DroneCommand(pos, 0));
	}

	int startAngle = 0;
	double step = (angle - startAngle) / commands.size();
	for (int i = 0; i < commands.size(); ++i) {
		commands[i].yaw = startAngle + i * step;
	}
	std::cout << "found a way" << std::endl;

	return true;

}

octomap::OcTreeKey operator +(const octomap::OcTreeKey& v1,
		const octomap::OcTreeKey& v2) {

	octomap::OcTreeKey ret(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]);
	return ret;

}

octomap::OcTreeKey operator -(const octomap::OcTreeKey& v1,
		const octomap::OcTreeKey& v2) {

	octomap::OcTreeKey ret(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
	return ret;
}

std::ostream& operator <<(std::ostream& os, const octomap::OcTreeKey& obj) {
	os << obj[0] << ", " << obj[1] << ", " << obj[2] << std::endl;
	return os;
}

void LSDWrapper::addNewCommands(std_msgs::Empty msg) {

	addingCommands = true;

	if (ptamWrapper->lastNavinfoReceived.state == 2) {
		// if drone landed...
		justTakingOff = true;

		std::stringstream c;
// 		c << "c clearCommands" << std::endl;
		c << "c newtakeoff";
		// c << "c start" << std::endl;
		std::cout << "sending: " << c.str();
		publish(c.str());

		publish("c start");
		addingCommands = false;
		return;
	}

	if (justTakingOff) {
		takenOffOnce = true;
		std::cout << "Sleeping 3 seconds, so that the drone can take of fully"
				<< std::endl;
		ros::Duration dur(3.0);
		dur.sleep();
		std::string commands[] = { "setReference $POSE$",
				"setInitialReachDist 300", "setStayWithinDist 300",
				"setStayTime 1.0", "lockScaleFP",

				"gotohov 0 0 0.5 0", "gotohov 0 0 -0.25 0", "gotohov 0 0 0.5 0"
						"gotohov 0 0 0 0", "gotohov 0 0 0 0" };

		/*"goto 0 0.1 0.5 0",
		 "goto 0 0.1 -0.25 0",
		 "goto 0 0.1 0.5 0",
		 "goto 0 0.1 -0.25 0",
		 "goto 0 0.1 0.5 0",
		 "goto 0 0.1 -0.25 0",
		 "goto 0 0.1 0.5 0",
		 "goto 0 0.1 -0.25 0",
		 "goto 0 0.1 0.5 0",
		 "goto 0 0.1 -0.25 0",

		 "goto 0 0.1 0 0"};*/
		// "goto 0 0 0 0"};
		for (int i = 0; i < sizeof(commands) / sizeof(std::string); ++i) {
			std::stringstream c;
			c << "c " << commands[i];
			publish(c.str());
		}

		startLSD();

		ros::Duration duration(6);
		duration.sleep();

		ROS_INFO("LSD-SLAM should be initialized now...");
		std::string commands2[] = { "setReference $POSE$",
				"setInitialReachDist 300", "setStayWithinDist 300",
				"setStayTime 1.5", "lockScaleFP",

				"goto 0 0 0.5 0", "goto 0 0 -0.25 0", "goto 0 0 0.5 0",
				"goto 0 0 -0.25 0", "goto 0 0 0.5 0", "goto 0 0 -0.25 0",
				"goto 0 0.1 -0.25 0" };
		//"goto 0 0.1 0 0"};

		for (int i = 0; i < sizeof(commands2) / sizeof(std::string); ++i) {
			std::stringstream c;
			c << "c " << commands2[i];
			publish(c.str());
		}

		// std::cout << "sending: " << c.str();
		// publish(c.str());

		justTakingOff = false;
		addingCommands = false;
		return;
	}

	if (commands.empty()) {
		// ros::spinOnce();
		std::cout << "Sleeping 3 seconds to wait for loop closures"
				<< std::endl;
		ros::Duration dur(3.0);
		dur.sleep();// sleep 1 second, so that the loop closures can happen...

		std::cout << "Done..." << std::endl;

		publish("c setReference $POSE$"); // set center to this, so the autopilot returns here, before a lag will happen.
		nextCenterYaw = 0.0;
		nextCenterPose = convertToAutopilot(lastCamToWorldWithoutTransform);

		doOctomap(std_msgs::Empty());

		if (currControl == LOOK_AROUND) {
			publish("c setStayTime 0.5");
			publish("c setStayWithinDist 0.4");
			publish("c setInitialReachDist 0.4");
		} else if (currControl == STAR_DISCOVERY) {
			publish("c setStayTime 0.4");
			publish("c setStayWithinDist 0.4");
			publish("c setStayTime 0.1");
		} else {
			publish("c setStayTime 0.5");
			publish("c setStayWithinDist 0.5");
			publish("c setStayTime 0.05");
		}
	}

	if (commands.empty()) {
		std::cout
				<< "PROBLEM: After creating the octomap there are still no commands!"
				<< std::endl;

		addingCommands = false;
		return;
	}

	// Note: the last command in the queue is the first one used!
	DroneCommand curr = commands.back();
	commands.pop_back();
	std::stringstream c;

	Eigen::Vector3f pos = convertToAutopilot(curr.position);

	if (currControl == LOOK_AROUND || currControl == PATH) {
		// Go to last position when lag is about to happen.

		Eigen::Vector3f realCenterPos = 0.5 * (nextCenterPose + pos);
		std::stringstream r;
		r << "c setReference " << realCenterPos.x() << " " << realCenterPos.y()
				<< " " << realCenterPos.z() << " "
				<< 0.5 * (nextCenterYaw + curr.yaw);
		publish(r.str());

		nextCenterPose = pos;
		nextCenterYaw = curr.yaw;
	} else if (currControl == STAR_DISCOVERY) {
		std::stringstream r;
		r << "c setReference " << nextCenterPose.x() << " "
				<< nextCenterPose.y() << " " << nextCenterPose.z() << " "
				<< nextCenterYaw;
		publish(r.str());
		nextCenterYaw = curr.yaw;
	}

	c << "c " << "gotoraw " << pos.x() << " " << pos.y() << " " << pos.z()
			<< " " << curr.yaw;

	std::cout << "sending command " << c.str() << std::endl;
	publish(c.str());

	addingCommands = false;

}

void LSDWrapper::generateLookAroundCommands(std::vector<DroneCommand>& output,
		int number, octomap::point3d position, double currentYaw) {

	double adding = 0.3 / ptamWrapper->getScale();
	for (int i = 0; i < (number + 2); ++i) {
		octomap::point3d thisPosition = position;
		thisPosition.y() += adding;
		adding = -adding;
		output.push_back(DroneCommand(thisPosition, 360 * i / number + currentYaw));
	}

	octomap::point3d thisPosition = position;
	thisPosition.y() += adding;
	output.push_back(DroneCommand(thisPosition, 0 + currentYaw));
	output.push_back(DroneCommand(position, 0 + currentYaw));

}

void LSDWrapper::addNewCommandsInOwnThread(std_msgs::Empty msg) {

	if (!addingCommands) {
		addingCommands = true;
		boost::thread t(boost::bind(&LSDWrapper::addNewCommands, this, msg));
	}

}

octomap::point3d LSDWrapper::convertCameraToDroneCenter(Sophus::Sim3f pos) {
	Eigen::Vector3f dir = (pos.rotationMatrix() * Eigen::Vector3f(0, 0, 1));

	std::cout << "oldPos: " << pos.translation() << std::endl;
	dir.normalize();
	// TODO(lsd): Make smaller
	Eigen::Vector3f nPos = pos.translation()
			+ (float) (0.1 / ptamWrapper->getScale()) * dir;

	std::cout << "newPos: " << nPos << std::endl;
	return eigenVecToOcto(nPos);
}

void LSDWrapper::addCommandsToQueue(std::vector<DroneCommand> c) {
	/*for(std::vector<DroneCommand>::reverse_iterator it = c.rbegin(); it != c.rend(); ++it)
	 {
	 commands.insert(commands.end(), *it);
	 }*/

	for (std::vector<DroneCommand>::iterator it = c.begin(); it != c.end();
			++it) {
		commands.insert(commands.begin(), *it);
	}

}

bool TimeMeasuring::init = false;
std::fstream TimeMeasuring::file;
FileCloser TimeMeasuring::closer;

TimeMeasuring::TimeMeasuring(std::string name) :
		name(name), start(ros::Time::now()) {
}

void TimeMeasuring::end() {
	ros::Duration dur = ros::Time::now() - start;
	std::stringstream output;
	output << name << " took " << dur.toSec() << std::endl;
	std::cout << output.str();
	if (!init) {
		std::cout << "Opening file for times..." << std::endl;
		init = true;
		std::string dir = "Flight 1";
		if (!boost::filesystem::exists(dir)) {
			boost::filesystem::create_directories(dir);
		}

		file.open("Flight 1/times.txt",
				std::fstream::out | std::fstream::trunc);
		closer.file = &file;

	}

	file << output.str() << std::endl;
}

FileCloser::FileCloser() :
		file(0) {
}

FileCloser::~FileCloser() {
	std::cout << "Closing file..." << std::endl;
	if (file)
		file->close();
}

// Like insertPointcloud, but for the scans in scanMiss, the endpoint shall not be updated as occupied, but left unknown.
void LSDWrapper::insertPointCloudWithMisses(octomap::OcTree& tree,
		const octomap::Pointcloud& scan, octomap::Pointcloud& scanMiss,
		const octomap::point3d& sensor_origin, double maxrange, bool lazy_eval,
		bool discretize, double *insertionTime, double *setCreationTime) {

	ros::Time t1 = ros::Time::now();
	octomap::KeySet free_cells, occupied_cells, occupied_unused;
	if (discretize) {
		tree.computeDiscreteUpdate(scanMiss, sensor_origin, free_cells,
				occupied_unused, maxrange);
		for (octomap::KeySet::iterator it = occupied_unused.begin();
				it != occupied_unused.end(); ++it) {
			free_cells.insert(*it);
		}
		tree.computeDiscreteUpdate(scan, sensor_origin, free_cells,
				occupied_cells, maxrange);
	} else {
		tree.computeUpdate(scanMiss, sensor_origin, free_cells, occupied_unused,
				maxrange);
		for (octomap::KeySet::iterator it = occupied_unused.begin();
				it != occupied_unused.end(); ++it) {
			free_cells.insert(*it);
		}
		tree.computeUpdate(scan, sensor_origin, free_cells, occupied_cells,
				maxrange);
	}

	if (setCreationTime) {
		*setCreationTime += (ros::Time::now() - t1).toSec();
	}

	ros::Time t2 = ros::Time::now();

	// insert data into tree  -----------------------
	for (octomap::KeySet::iterator it = free_cells.begin();
			it != free_cells.end(); ++it) {
		tree.updateNode(*it, false, lazy_eval);
	}
	for (octomap::KeySet::iterator it = occupied_cells.begin();
			it != occupied_cells.end(); ++it) {
		tree.updateNode(*it, true, lazy_eval);
	}

	if (insertionTime) {
		*insertionTime += (ros::Time::now() - t2).toSec();
	}

}

void LSDWrapper::paintFree(octomap::OcTree& t, octomap::OcTreeKey pos, int dx,
		int dy, int dz) {
	octomap::OcTreeKey kMin = getOctomapMinKey(t);
	octomap::OcTreeKey kMax = getOctomapMaxKey(t);
	// TODO: watch out that you don't get through that...
	for (int x = pos[0] - dx; x <= pos[0] + dx; ++x) {
		for (int y = pos[1] - dy; y <= pos[1] + dy; ++y) {
			for (int z = pos[2] - dz; z <= pos[2] + dz; ++z) {
				octomap::OcTreeKey k(x, y, z);
				t.updateNode(k, false, true);
				t.updateNode(k, false, true);
				t.updateNode(k, false, true);
			}
		}
	}

	t.updateInnerOccupancy();

}
