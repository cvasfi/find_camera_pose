#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "lsd_slam_viewer/keyframeMsg.h"
#include <sstream>
#include <fstream>
#include <math.h>
#include <Eigen/Geometry>
#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"
#include <boost/filesystem.hpp>
    #include <rosbag/bag.h>
#include <boost/foreach.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
    #include <rosbag/view.h>
#include <ctime>

#include "matching.h"

struct InputPointDense {
        float idepth;
        float idepth_var;
        unsigned char color[4];
};

Vector<cv::Point2f> queryPoints;
Vector<cv::Point2f> match2dPoints;
Vector<cv::Point3f> match3dPoints;
int match_ID;
lsd_slam_viewer::keyframeMsgConstPtr matchedFrame;

rosbag::Bag frameBag;
cv_bridge::CvImagePtr cv_ptr;
//int i=0;
  std::ofstream imageList;

void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    std::ostringstream oss;
  try
  {
    cv_ptr    = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
    oss << "src/find_camera_pose/images/" << msg->header.frame_id.data()<<".png";
    cv::imwrite( oss.str(), cv_ptr->image );
    imageList <<msg->header.frame_id.data()<<".png"<<std::endl;

    //savepcl
//    i++;

}

void frameCB(const lsd_slam_viewer::keyframeMsgConstPtr& msg)
{

    frameBag.write("frames", ros::Time::now(), *msg);


}


void calculate3DPoints(Vector<cv::Point2f>& mCoordinates, lsd_slam_viewer::keyframeMsgConstPtr& frame){
    InputPointDense *points = (InputPointDense*)frame->pointcloud.data();
    Sophus::Sim3f camToWorld;
    memcpy(camToWorld.data(), frame->camToWorld.data(), 7*sizeof(float));
    int width=frame->width;
    int height=frame->height;

    float my_scale = camToWorld.scale(); // TODO lsd: maybe add maximum-likelyhood scaling factor...
    // float my_scaledTH = exp10(-2.5);
    float my_scaledTH = exp10(-3.0);
    float my_scaledTH2 = exp10(-2.0);
    int my_minNearSupport = 7;
    float my_absTH = exp10(-1.0);
    float my_absTH2 = exp10(-1.0);
    int sparsify1=2;
    int sparsify2=5;
    double maxDist = 0;
    int otherMinNearSupport = 9;

    int fxi=1/frame->fx;
    int fyi=1/frame->fy;
    int cxi=-frame->cx/frame->fx;
    int cyi=-frame->cy/frame->fy;

    for(size_t i;i<mCoordinates.size();i++){
        int x=mCoordinates[i].x;
        int y=mCoordinates[i].y;

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
        cv::Point3f point;
        point.x = (x * fxi + cxi) * depth;
        point.y = (y * fyi + cyi) * depth;
        point.z = depth;

        match3dPoints.push_back(point);

        if (depth > maxDist) {
                maxDist = depth;
        }

       // Eigen::Vector3f vec(posX, posY, posZ);
       // Eigen::Vector3f realPos = camToWorld * vec;
    }

}

void getMatchedFrame(int fid){
   frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Read);

   rosbag::View view(frameBag, rosbag::TopicQuery("frames"));
   lsd_slam_viewer::keyframeMsg::ConstPtr frame;
   BOOST_FOREACH(rosbag::MessageInstance const m, view)
   {
       frame = m.instantiate<lsd_slam_viewer::keyframeMsg>();
       if (frame != NULL)
           std::cout<<"read frame with id: "<<frame->id<<std::endl;
       if(frame->id==fid){
           std::cout<<"I FOUND THE BEST MATCH!!!! : "<<frame->id<<std::endl;
           matchedFrame=frame;
       }

   }
   frameBag.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_camera_pose");

  ros::NodeHandle n;
  ros::Time recordBegin = ros::Time::now();
  ros::Time recordEnd = ros::Time::now();
  ros::Duration recordTime(20);    //record for 3 minutes
  ros::Rate r(10); // 10 hz

  imageList.open("src/find_camera_pose/images/trainImageList.txt", std::ios_base::app);
  frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Write);

  ros::Subscriber subImages = n.subscribe("/lsd_slam/keyImages", 1000, imageCB);
  ros::Subscriber subFrames = n.subscribe("/lsd_slam/keyframes", 1000, frameCB);

  //mtmi()
  while(ros::ok() && (recordEnd-recordBegin)<recordTime){
      ros::spinOnce();
      r.sleep();
      recordEnd = ros::Time::now();
  }
  subImages.shutdown();
  subFrames.shutdown();
  frameBag.close();
  //record is over

    match_ID=findMatch();
    queryPoints=getQueryCoordinates();
    match2dPoints=getMatchCoordinates();
   // calculate3DPoints(match2dPoints,match3dPoints);
 // ros::spin();
    getMatchedFrame(match_ID);
    calculate3DPoints(match2dPoints,matchedFrame);

    std::cout<<"size of 2d points: "<<match2dPoints.size()<<std::endl;
    std::cout<<"size of 3d points: "<<match3dPoints.size()<<std::endl;

    std::cout<<"dif is: "<<(recordEnd-recordBegin);
  return 0;
}
