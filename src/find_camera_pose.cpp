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
#include "ControlNode.h"
#include "DroneController.h"
#include "TooN/se3.h"
#include "HelperFunctions.h"
#include "tum_ardrone/filter_state.h"
#include "matching.h"


struct InputPointDense {
        float idepth;
        float idepth_var;
        unsigned char color[4];
};

vector<cv::Point2f> queryPoints;
vector<cv::Point2f> selected2dPoints;
vector<cv::Point2f> match2dPoints;
vector<cv::Point3f> match3dPoints;
int match_ID;
lsd_slam_viewer::keyframeMsgConstPtr matchedFrame;

rosbag::Bag frameBag;
rosbag::Bag KalmanFilterBag; //Hide
float x_kf, y_kf, z_kf, roll_kf, pitch_kf, yaw_kf, scale_kf;
float scale;

cv_bridge::CvImagePtr cv_ptr;
ofstream imageList;

void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    ostringstream oss;
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
    imageList <<msg->header.frame_id.data()<<".png"<<endl;
}

void frameCB(const lsd_slam_viewer::keyframeMsgConstPtr& msg)
{
    frameBag.write("frames", ros::Time::now(), *msg);
}

//Hide
void KalmanFilterCB(const tum_ardrone::filter_stateConstPtr& msg){
    KalmanFilterBag.write("Pose_KalmanFilter", ros::Time::now(), *msg);
}

void calculate3DPoints(std::vector<cv::Point2f>& mCoordinates, lsd_slam_viewer::keyframeMsgConstPtr& frame){
    InputPointDense *points = (InputPointDense*)frame->pointcloud.data();
    Sophus::Sim3f camToWorld;
    memcpy(camToWorld.data(), frame->camToWorld.data(), 7*sizeof(float));
    int width=frame->width;
    int height=frame->height;

    float my_scale = camToWorld.scale(); // TODO lsd: maybe add maximum-likelyhood scaling factor...
    float my_scaledTH = exp10(-3.0);
    float my_scaledTH2 = exp10(-2.0);
    int my_minNearSupport = 7;
    float my_absTH = exp10(-1.0);
    float my_absTH2 = exp10(-1.0);
    int sparsify1=2;
    int sparsify2=5;
    double maxDist = 0;
    int otherMinNearSupport = 9;

    float fxi=1/frame->fx;
    float fyi=1/frame->fy;
    float cxi=-frame->cx/frame->fx;
    float cyi=-frame->cy/frame->fy;
    int z=0;

    for(size_t i;i<mCoordinates.size();i++){
        z++;
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

        //Calculate Position in camera coordinate
        cv::Point3f point;
        point.x = my_scale*scale*(x * fxi + cxi) * depth;
        point.y = my_scale*scale*(y * fyi + cyi) * depth;
        point.z = my_scale*scale*depth;
        selected2dPoints.push_back(mCoordinates[i]);
        match3dPoints.push_back(point);

        if (depth > maxDist) {
                maxDist = depth;
        }


    }
    cout<<"scale, my_scale"<<scale<<", "<<my_scale<<endl;

}


void getMatchedFrame(int fid){
    frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Read);

    rosbag::View view(frameBag, rosbag::TopicQuery("frames"));
    lsd_slam_viewer::keyframeMsg::ConstPtr frame;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
       frame = m.instantiate<lsd_slam_viewer::keyframeMsg>();
       if (frame != NULL)
           cout<<"read frame with id: "<<frame->id<<endl;
       if(frame->id==fid){
           cout<<"I FOUND THE BEST MATCH!!!! : "<<frame->id<<endl;
           matchedFrame=frame;
       }

    }
    frameBag.close();
    }

void getPositionFromKalmanFilter(){
   KalmanFilterBag.open("src/find_camera_pose/KalmanFilter/KalmanFilter.bag", rosbag::bagmode::Read);
   bool match;
   double matchtime_kf;
   double matchtime_LSD;
   lsd_slam_viewer::keyframeMsgConstPtr copy;
   copy = matchedFrame;
   rosbag::View view(KalmanFilterBag, rosbag::TopicQuery("Pose_KalmanFilter"));
   tum_ardrone::filter_state::ConstPtr framekf;
   BOOST_FOREACH(rosbag::MessageInstance const m, view)
   {
       framekf = m.instantiate<tum_ardrone::filter_state>();
       if (framekf != NULL)
//           std::cout<<"read Time stamp: "<<frame->header.stamp<<std::endl;
//           std::cout<<"Matched Time stamp: "<<ros::Time(matchedFrame->time)<<std::endl;

//           frame->header.stamp.toSec()==matchedFrame->time
//           frame->header.stamp-ros::Time(matchedFrame->time)
       if(abs(framekf->header.stamp.toSec()-copy->time)<0.02){
           match =true;
           matchtime_LSD=framekf->header.stamp.toSec();
           matchtime_kf=copy->time;
//           std::cout<<"BEST MATCH Time Stamp in Kalman Filter : "<<frame->header.stamp.toSec()<<std::endl << copy->time<<endl;
           x_kf=framekf->x; y_kf=framekf->y; z_kf=framekf->z;
           roll_kf=framekf->roll; pitch_kf=framekf->pitch; yaw_kf=framekf->yaw;
           scale = framekf->scale;
       }
   }
//   std::cout<<"BEST MATCH Time Stamp in Kalman Filter : "<<std::setprecision(20)<<frame->header.stamp.toSec()<<std::endl <<std::setprecision(20)<< copy->time<<endl;
   if(match){
   std::cout<<"match!"<<std::endl;
   cout<<"matchtime_kf: " <<std::setprecision(20)<<matchtime_kf<<endl;
   cout<<"matchtime_LSD: " <<std::setprecision(20)<<matchtime_LSD<<endl;
   }
    KalmanFilterBag.close();
}



int main(int argc, char **argv)
 {
    ros::init(argc, argv, "find_camera_pose");

    ros::NodeHandle n;
    ros::Time recordBegin = ros::Time::now();
    ros::Time recordEnd = ros::Time::now();
    ros::Duration recordTime(35);    //record for 3 minutes
    ros::Rate r(10); // 10 hz

    imageList.open("src/find_camera_pose/images/trainImageList.txt", ios_base::app);
    frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Write);
    KalmanFilterBag.open("src/find_camera_pose/KalmanFilter/KalmanFilter.bag", rosbag::bagmode::Write);

    undistortImage();
    ros::Subscriber subImages = n.subscribe("/lsd_slam/keyImages", 1000, imageCB);
    ros::Subscriber subFrames = n.subscribe("/lsd_slam/keyframes", 1000, frameCB);
    ros::Subscriber subKalmanFilter = n.subscribe("/ardrone/predictedPose", 1000, KalmanFilterCB);


    //mtmi()
    while(ros::ok() && (recordEnd-recordBegin)<recordTime){
      ros::spinOnce();
      r.sleep();
      recordEnd = ros::Time::now();
    }
    subImages.shutdown();
    subFrames.shutdown();
    subKalmanFilter.shutdown();
    frameBag.close();
    KalmanFilterBag.close();
    //record is over

    match_ID=findMatch();
    queryPoints=getQueryCoordinates();
    match2dPoints=getMatchCoordinates();
    cout<<"MatchID is : "<<match_ID<<endl;


    getMatchedFrame(match_ID);
    getPositionFromKalmanFilter();
    calculate3DPoints(match2dPoints,matchedFrame);




    //Key Frame position in world coordinate
    TooN::SE3<> KeyFrameToWorldSE3;
    KeyFrameToWorldSE3.get_translation()=TooN::makeVector(x_kf,y_kf,z_kf);
    KeyFrameToWorldSE3.get_rotation() = rpy2rod(roll_kf, pitch_kf, yaw_kf);
    cout<<"size of 2d points: "<<match2dPoints.size()<<endl;
    cout<<"size of 3d points: "<<match3dPoints.size()<<endl;
    cout<<"size of selected 2d points: "<<selected2dPoints.size()<<endl;
    cout<<"Kalman Filter Position: "<<x_kf<<","<<y_kf<<","<<z_kf<<endl;

    //  SolvePnP
    cv::Mat K = (cv::Mat_<float>(3, 3) << matchedFrame->fx, 0, matchedFrame->cx, 0, matchedFrame->fy, matchedFrame->cy, 0, 0, 1);
    cv::Mat distortion = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    cout <<"match3Dpoint"<< match3dPoints << endl;

    cv::solvePnPRansac(match3dPoints,selected2dPoints,K,distortion,rotation_vector,translation_vector);
//    cv::solvePnP(match3dPoints,selected2dPoints,K,distortion,rotation_vector,translation_vector);

    cout<<"translation is: "<<translation_vector<<endl;
    cout<<"Rotation is: "<<rotation_vector<<endl;


    //90deg Rotation (PTAMWrapper.cpp, Handleframe)
    TooN::SE3<> LSDSLAM_ResultSE3; //Key Frame to Hand camera
    LSDSLAM_ResultSE3.get_translation()=TooN::makeVector(translation_vector.at<double>(0,0),translation_vector.at<double>(1,0),translation_vector.at<double>(2,0));
    cv::Mat rmat;
    cv::Rodrigues(rotation_vector, rmat);
    TooN::Matrix<3,3> mat;
    mat(0,0)=rmat.at<double>(0,0); mat(0,1)=rmat.at<double>(0,1); mat(0,2)=rmat.at<double>(0,2);
    mat(1,0)=rmat.at<double>(1,0); mat(1,1)=rmat.at<double>(1,1); mat(1,2)=rmat.at<double>(1,2);
    mat(2,0)=rmat.at<double>(2,0); mat(2,1)=rmat.at<double>(2,1); mat(2,2)=rmat.at<double>(2,2);
    LSDSLAM_ResultSE3.get_rotation()=mat;
    cout <<"translation_vector is"<< endl<< translation_vector <<endl << "get_traslation is "<<endl<<LSDSLAM_ResultSE3.get_translation() <<  endl; //For check
    cout <<"rmat is"<< endl<< rmat <<endl << "mat is"<<endl<<mat <<  endl; //For check

    TooN::SE3<> LSDSLAM_ResultSE3_camToKeyFrame;
    LSDSLAM_ResultSE3_camToKeyFrame = LSDSLAM_ResultSE3.inverse();
    double testRoll = 0;
    double testPitch = 0;
    double testYaw = 0;
    rod2rpy(LSDSLAM_ResultSE3_camToKeyFrame.get_rotation(), &testRoll, &testPitch, &testYaw);
    testPitch += 90.0; //testRoll += 90.0; //    testPitch += 90.0;
    LSDSLAM_ResultSE3_camToKeyFrame.get_rotation() = rpy2rod(testRoll, testPitch, testYaw);
    rod2rpy(LSDSLAM_ResultSE3_camToKeyFrame.get_rotation(), &testRoll, &testPitch, &testYaw);



    //Calculate camera position in world coordinate
    TooN::Vector<3> origin = TooN::makeVector(0,0,0);
    TooN::Vector<3> result = KeyFrameToWorldSE3*LSDSLAM_ResultSE3_camToKeyFrame*origin;
    cout<<"Kalman Filter Position: "<<x_kf<<","<<y_kf<<","<<z_kf<<endl;
    cout <<"Camera position in World Coordinate is "<<result <<  endl; //For check
    cout <<"Yaw angle is "<<testYaw <<  endl<<endl;
    cout << "Command is"<< endl;

    char buf[200];
    snprintf(buf,200,"gotoraw %.3f %.3f %.3f %.3f", result[0],result[1],result[2],testYaw);
    cout <<buf<<endl;


//    //Publish (DroneController.cpp, DroneController::setTarget(target))
//    ros::Publisher tum_ardrone_pub;
//    string command_channel;
//    command_channel = n.resolveName("tum_ardrone/com");
//    tum_ardrone_pub	   = n.advertise<std_msgs::String>(command_channel,50);
//    char buf[200];
//    snprintf(buf,200,"New Target: xyz = %.3f, %.3f, %.3f,  yaw=%.3f", result[0],result[1],result[2],testYaw);
//    string c=string("u l ") + buf;
////    while (ros::ok())
////    {
//    std_msgs::String s;
//    s.data = c.c_str();
//    pthread_mutex_t tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
//    pthread_mutex_lock(&tum_ardrone_CS);
//    tum_ardrone_pub.publish(s);
//    pthread_mutex_unlock(&tum_ardrone_CS);
////    }

  return 0;
}




