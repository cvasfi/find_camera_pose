#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
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
#include "TooN/se3.h"
#include "HelperFunctions.h"
#include "tum_ardrone/filter_state.h"
#include "matching.h"
#include <boost/assign/std/vector.hpp>

#include <sys/types.h>
#include <unistd.h>

#include <stdlib.h>

#define INITIAL 0
#define INITIALIZE 1
#define EXPLORING 2
#define EXPLORED 3
#define WAITING 4
#define READY_TO_GO 5

using namespace boost::assign;

struct InputPointDense {
        float idepth;
        float idepth_var;
        unsigned char color[4];
};

class find_camera_pose_node{
private:
int drone_state;
int drone_prev_state;
ros::NodeHandle n;
ros::Subscriber noCommandSub;
ros::Publisher tum_ardrone_pub;
std::string command_channel;
ros::Duration recordTime;    //record for 3 minutes
ros::Duration rec1;
ros::Duration rec2;
typedef boost::function<void(std::string)> PublishCommandFunction;
PublishCommandFunction publish;

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
double testRoll;
double testPitch;
double testYaw;
//double maxFrameDelay=0;
//double maxKFDelay=0;
ros::Time recordBegin;
ros::Subscriber subImages;
ros::Subscriber subFrames;
ros::Subscriber subKalmanFilter;
cv_bridge::CvImagePtr cv_ptr;
ofstream imageList;
ros::Publisher noCommandsPublisher;
int lastDroneState;
bool canRecord;

std::vector<std::string> initializeCommands;
std::vector<std::string> simpleExploreCommands;
std::vector<std::string> _360exploreCommands;
std::vector<std::string> selectedCommands;
int exploration_mode;

pid_t pID;

cv::Mat rotation_vector; // Rotation in axis-angle form
cv::Mat translation_vector;
TooN::Vector<3> result;

std::string finalCommand;

int RecTime;

void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    if(msg->header.stamp.toSec()>=recordBegin.toSec()&& canRecord){

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
}


void frameCB(const lsd_slam_viewer::keyframeMsgConstPtr& msg)
{
//    if(msg->time>=recordBegin.toSec())  //only write frames that have been processed after the application got started
        frameBag.write("frames", ros::Time::now(), *msg);
}

//Hide
void KalmanFilterCB(const tum_ardrone::filter_stateConstPtr& msg){
    KalmanFilterBag.write("Pose_KalmanFilter", ros::Time::now(), *msg);
    lastDroneState=msg->droneState;
}

void calculate3DPoints(std::vector<cv::Point2f>& mCoordinates,std::vector<cv::Point2f>& qCoordinates, lsd_slam_viewer::keyframeMsgConstPtr& frame){
    cout<<"> calculating 3D points........"<<endl;
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
        selected2dPoints.push_back(qCoordinates[i]);
        match3dPoints.push_back(point);

        if (depth > maxDist) {
                maxDist = depth;
        }


    }
//    cout<<"scale, my_scale"<<scale<<", "<<my_scale<<endl;

}


void getMatchedFrame(int fid){
    frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Read);
    cout<<"> Searching for the matched frame........"<<endl;
    rosbag::View view(frameBag, rosbag::TopicQuery("frames"));
    lsd_slam_viewer::keyframeMsg::ConstPtr frame;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
       frame = m.instantiate<lsd_slam_viewer::keyframeMsg>();
       if (frame != NULL){
//           cout<<"read frame with id: "<<frame->id<<endl;
       if(frame->id==fid){
           cout<<"Best match is : "<<frame->id<<"timestamp is: "<<frame->time<<endl;
           matchedFrame=frame;
       }
       }

    }
    frameBag.close();
    }

void getPositionFromKalmanFilter(){
    cout<<"> searching for keyFrame's position........"<<endl;
   KalmanFilterBag.open("src/find_camera_pose/KalmanFilter/KalmanFilter.bag", rosbag::bagmode::Read);
   bool match=false;
   double matchtime_kf;
   double matchtime_LSD;
   lsd_slam_viewer::keyframeMsgConstPtr copy;
   copy = matchedFrame;
   rosbag::View view(KalmanFilterBag, rosbag::TopicQuery("Pose_KalmanFilter"));
   tum_ardrone::filter_state::ConstPtr framekf;
   double time_stamp_error=0.02;
   double min_difference=1000;
   double min_timeStamp=0;
   int it=0;
       BOOST_FOREACH(rosbag::MessageInstance const m, view)
       {

           framekf = m.instantiate<tum_ardrone::filter_state>();
           if (framekf != NULL){
//               std::cout<<"read Time stamp: "<<framekf->header.stamp<<std::endl;
           if(min_difference>abs(framekf->header.stamp.toSec()-matchedFrame->time))
            {
               min_difference=abs(framekf->header.stamp.toSec()-matchedFrame->time);
               min_timeStamp=framekf->header.stamp.toSec();
               if(min_difference<2){
                   match=true;
                   matchtime_LSD=framekf->header.stamp.toSec();
                   matchtime_kf=copy->time;
        //           std::cout<<"BEST MATCH Time Stamp in Kalman Filter : "<<frame->header.stamp.toSec()<<std::endl << copy->time<<endl;
                   x_kf=framekf->x; y_kf=framekf->y; z_kf=framekf->z;
                   roll_kf=framekf->roll; pitch_kf=framekf->pitch; yaw_kf=framekf->yaw;
                   scale = framekf->scale;
               }
           }
           }

       }
   cout << "time_stamp_error is " << min_difference << endl;

   if(match){
   std::cout<<"match!"<<std::endl;
   cout<<"matchtime_kf: " <<std::setprecision(20)<<matchtime_kf<<endl;
   cout<<"matchtime_LSD: " <<std::setprecision(20)<<matchtime_LSD<<endl;
   }
   else{
       cout<<"Not match!"<<endl;

   }
    KalmanFilterBag.close();
}


pthread_mutex_t tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
void publishCommand(ros::NodeHandle n, ros::Publisher tum_ardrone_pub, std::string command_channel, std::string c)
{
    std_msgs::String s;
    s.data = c.c_str();
//    pthread_mutex_lock(&tum_ardrone_CS);
    tum_ardrone_pub.publish(s);
//    pthread_mutex_unlock(&tum_ardrone_CS);
}

void noCommandCB(std_msgs::Empty msg){

    cout<<"nocommand called"<<std::endl;

    command_channel = n.resolveName("tum_ardrone/com");

    switch(drone_state){
    case INITIAL :
    {
        cout<<"drone state is1: "<<drone_state<<std::endl;
//        ros::Duration d1(0.1);
//        ros::Time r1=ros::Time::now();
//        ros::Time r2=ros::Time::now();
//        ros::Rate rate1(100);
//        while(ros::ok() && (r2-r1)<d1){
////          ros::spinOnce();
//          rate1.sleep();
//          r2 = ros::Time::now();
//          if(lastDroneState == 2 || lastDroneState == 0)
//{
//                      publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
//                      publishCommand(n,tum_ardrone_pub,command_channel,"c newtakeoff");
//                      publishCommand(n,tum_ardrone_pub,command_channel,"c start");
//        }
//        d1.sleep();
//        }


        publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
        publishCommand(n,tum_ardrone_pub,command_channel,"c newtakeoff");
        publishCommand(n,tum_ardrone_pub,command_channel,"c start");

        drone_state=INITIALIZE;
        drone_prev_state=INITIAL;
        break;
    }
    case INITIALIZE:
        {
//        pID = fork();
//        if (pID == 0)                // child
//        {
//            cout<<"drone state is: "<<drone_state<<std::endl;
//            system("exec roslaunch bebop_converter lsd_slam.launch ");

//         }
//         else if (pID < 0)            // failed to fork
//         {
//             cerr << "Failed to fork" << endl;
//             exit(1);
//             // Throw exception
//         }
//         else                                   // parent
//         {
//           // Code only executed by parent process
//         }
        cout<<"time1 "<<ros::Time::now()<<endl;
//        ros::Duration r(3);    //record for 3 minutes
//        r.sleep();
        cout<<"time2 "<<ros::Time::now()<<endl;
        cout<<"drone state is: "<<drone_state<<std::endl;
        publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
        for (int i = 0; i < initializeCommands.size(); ++i) {
            std::stringstream c;
            c << "c " << initializeCommands[i];
            publishCommand(n,tum_ardrone_pub,command_channel,c.str());
        }
        publishCommand(n,tum_ardrone_pub,command_channel,"c start");
//        r.sleep();

        drone_prev_state=INITIALIZE;
        drone_state=EXPLORING;
    }
    case EXPLORING :
    {
//        cout<<"time1 "<<ros::Time::now()<<endl;
//        ros::Duration r(4);
//        r.sleep();
//        cout<<"time1 "<<ros::Time::now()<<endl;
        cout<<"drone state is: "<<drone_state<<std::endl;
        canRecord=true;
//        recordBegin=ros::Time::now();
        publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
        for (int i = 0; i < selectedCommands.size(); ++i) {
            std::stringstream c;
            c << "c " << selectedCommands[i];
            publishCommand(n,tum_ardrone_pub,command_channel,c.str());
        }
        publishCommand(n,tum_ardrone_pub,command_channel,"c start");
        drone_prev_state=EXPLORING;
        drone_state=EXPLORED;
        cout<<"sent the exploration command. state is now explored."<<std::endl;
        break;
    }
    case EXPLORED :
    {
        ros::Rate rr(10);
        cout<<"drone state is: "<<drone_state<<std::endl;
        publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
        publishCommand(n,tum_ardrone_pub,command_channel,"c land");
        publishCommand(n,tum_ardrone_pub,command_channel,"c start");
        drone_prev_state=EXPLORED;
        drone_state=WAITING;
        break;
    }
    case WAITING :
    {
        cout<<"drone state is: "<<drone_state<<std::endl;
        drone_prev_state=WAITING;
        break;
    }
    case READY_TO_GO :
    {
        cout<<"drone state is: "<<drone_state<<std::endl;
        cout<<"prev drone state is: "<<drone_prev_state<<std::endl;
        if(drone_prev_state==EXPLORED || drone_prev_state==WAITING)
        {
            publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
            publishCommand(n,tum_ardrone_pub,command_channel,"c newtakeoff");
            publishCommand(n,tum_ardrone_pub,command_channel,"c start");
            drone_prev_state=READY_TO_GO;
            drone_state=READY_TO_GO;
        }
        else
        {            cout<<"sending the final command."<<std::endl;

            publishCommand(n,tum_ardrone_pub,command_channel,"c clearCommands");
            publishCommand(n,tum_ardrone_pub,command_channel, finalCommand);
            publishCommand(n,tum_ardrone_pub,command_channel,"c start");
            drone_prev_state=READY_TO_GO;
            drone_state=WAITING;
        }
    //    publishCommand(n,tum_ardrone_pub,command_channel,"c land");

        break;
    }
    default :
        drone_state=WAITING;
        break;

    }


}

void calculateCommand(){
    TooN::SE3<> KeyFrameToWorldSE3;
    KeyFrameToWorldSE3.get_translation()=TooN::makeVector(x_kf,y_kf,z_kf);
    KeyFrameToWorldSE3.get_rotation() = rpy2rod(roll_kf, pitch_kf, yaw_kf);
    //90deg Rotation (PTAMWrapper.cpp, Handleframe)
        //Key Frame to Hand camera in LSD coordinate
        TooN::SE3<> LSDSLAM_;
        LSDSLAM_.get_translation()=TooN::makeVector(translation_vector.at<double>(0,0),translation_vector.at<double>(1,0),translation_vector.at<double>(2,0));
        cv::Mat rmat;
        cv::Rodrigues(rotation_vector, rmat);
        TooN::Matrix<3,3> mat;
        mat(0,0)=rmat.at<double>(0,0); mat(0,1)=rmat.at<double>(0,1); mat(0,2)=rmat.at<double>(0,2);
        mat(1,0)=rmat.at<double>(1,0); mat(1,1)=rmat.at<double>(1,1); mat(1,2)=rmat.at<double>(1,2);
        mat(2,0)=rmat.at<double>(2,0); mat(2,1)=rmat.at<double>(2,1); mat(2,2)=rmat.at<double>(2,2);
        LSDSLAM_.get_rotation()=mat;
    //    cout << "Rotation matrix is"<< endl<<mat<<endl<<endl; //For check

        //Hand camera to Key Frame in LSD coordinate
        TooN::SE3<> LSDSLAM__camToKeyFrame;
        LSDSLAM__camToKeyFrame = LSDSLAM_.inverse();
    //    cout<<"Translation camToKeyFrame in LSD coordinate is: "<<LSDSLAM__camToKeyFrame.get_translation()<<endl<<endl;


        //Hand camera to Key Frame in World coordinate
        TooN::SE3<> LSDSLAM__camToKeyFrame_KF;
        TooN::Matrix<3,3> rot_camTokeyFrame=LSDSLAM__camToKeyFrame.get_rotation().get_matrix();
        cv::Mat rmat_camToKeyframe=(cv::Mat_<double>(3,3)<<0,0,0,0,0,0,0,0,0);
        cv::Mat rvec_camToKeyframe;
        rmat_camToKeyframe.at<double>(0,0)=rot_camTokeyFrame(0,0); rmat_camToKeyframe.at<double>(0,1)=rot_camTokeyFrame(0,1); rmat_camToKeyframe.at<double>(0,2)=rot_camTokeyFrame(0,2);
        rmat_camToKeyframe.at<double>(1,0)=rot_camTokeyFrame(1,0); rmat_camToKeyframe.at<double>(1,1)=rot_camTokeyFrame(1,1); rmat_camToKeyframe.at<double>(1,2)=rot_camTokeyFrame(1,2);
        rmat_camToKeyframe.at<double>(2,0)=rot_camTokeyFrame(2,0); rmat_camToKeyframe.at<double>(2,1)=rot_camTokeyFrame(2,1); rmat_camToKeyframe.at<double>(2,2)=rot_camTokeyFrame(2,2);
        cv::Rodrigues(rmat_camToKeyframe, rvec_camToKeyframe); //Transform rotation matrix to rotation vector
        cv::Mat rotation_vector_World = (cv::Mat_<double>(3,1)<<rvec_camToKeyframe.at<double>(0,0),rvec_camToKeyframe.at<double>(2,0),-rvec_camToKeyframe.at<double>(1,0)); //Add 90 degree rotation
        cv::Mat rmat_World;
        cv::Rodrigues(rotation_vector_World, rmat_World); //Transform rotation vector to rotation matrix again
        TooN::Matrix<3,3> mat_world; //Rotation matrix in world coordinate
        mat_world(0,0)=rmat_World.at<double>(0,0); mat_world(0,1)=rmat_World.at<double>(0,1); mat_world(0,2)=rmat_World.at<double>(0,2);
        mat_world(1,0)=rmat_World.at<double>(1,0); mat_world(1,1)=rmat_World.at<double>(1,1); mat_world(1,2)=rmat_World.at<double>(1,2);
        mat_world(2,0)=rmat_World.at<double>(2,0); mat_world(2,1)=rmat_World.at<double>(2,1); mat_world(2,2)=rmat_World.at<double>(2,2);
        LSDSLAM__camToKeyFrame_KF.get_rotation()=mat_world; //Rotation matrix in world coordinate
        LSDSLAM__camToKeyFrame_KF.get_translation()=TooN::makeVector((LSDSLAM__camToKeyFrame.get_translation())[0],(LSDSLAM__camToKeyFrame.get_translation()[2]),-(LSDSLAM__camToKeyFrame.get_translation())[1]); //Translation matrix in world coordinate
    //    cout<<"Translation camToKeyFrame in KeyFrame coordinate is: "<<LSDSLAM__camToKeyFrame_KF.get_translation()<<endl<<endl;



        //Calculate camera position in world coordinate
        TooN::Vector<3> origin = TooN::makeVector(0,0,0);
        result = KeyFrameToWorldSE3*LSDSLAM__camToKeyFrame_KF*origin;
    //    cout <<"(KeyFrameToWorldSE3*LSDSLAM__camToKeyFrame_World).get_translation() "<< endl<<(KeyFrameToWorldSE3*LSDSLAM__camToKeyFrame_World).get_translation() <<endl;

        rod2rpy((KeyFrameToWorldSE3*LSDSLAM__camToKeyFrame_KF).get_rotation(), &testRoll, &testPitch, &testYaw);


        cout<<"Kalman Filter Position (x,y,z,yaw): "<<endl<<x_kf<<","<<y_kf<<","<<z_kf<<","<<yaw_kf<<endl;
        cout <<"Camera position in World Coordinate (x,y,z,yaw): "<<endl<<result << "," << testYaw<< endl; //For check
        cout <<"Dıfference is (x,y,z,yaw): "<<endl<<result[0]-x_kf << "," <<result[1]-y_kf << " , "<<result[2]-z_kf <<" , "<<  testYaw - yaw_kf<< endl; //For chec
}
public:

find_camera_pose_node(ros::NodeHandle nh) :
    n(nh)
  {
    system("exec rm -r src/find_camera_pose/images/*");
    system("exec rm -r src/find_camera_pose/results/*");
    canRecord==false;
    drone_state=INITIAL;
    drone_prev_state=INITIAL;
    testRoll = 0;
    testPitch = 0;
    testYaw = 0;
    lastDroneState=0;
    imageList.open("src/find_camera_pose/images/trainImageList.txt", ios_base::app);
    frameBag.open("src/find_camera_pose/frames/frames.bag", rosbag::bagmode::Write);
    KalmanFilterBag.open("src/find_camera_pose/KalmanFilter/KalmanFilter.bag", rosbag::bagmode::Write);

    noCommandSub = nh.subscribe("/tum_ardrone/nocommands",1, &find_camera_pose_node::noCommandCB, this );
    subImages = nh.subscribe("/lsd_slam/keyImages", 1000, &find_camera_pose_node::imageCB,this);
    subFrames = nh.subscribe("/lsd_slam/keyframes", 1000, &find_camera_pose_node::frameCB,this);
    subKalmanFilter = nh.subscribe("/ardrone/predictedPose", 1000, &find_camera_pose_node::KalmanFilterCB,this);
    rec1=ros::Duration(1);
    rec2=ros::Duration(10);
    command_channel = nh.resolveName("tum_ardrone/com");
    tum_ardrone_pub = nh.advertise<std_msgs::String>(command_channel,50);
    noCommandsPublisher= n.advertise<std_msgs::Empty>("/tum_ardrone/nocommands", 1);

    initializeCommands += "setReference $POSE$",
                        "setInitialReachDist 0.3",
                        "setStayWithinDist 0.4",
                                       "setStayTime 1",

                                       "gotohov 0 0 0.50 0",
                                        "gotohov 0 0 -0.25 0",
                                        "gotohov 0 0 0.50 0"
                                        "gotohov 0 0 0 0";

    simpleExploreCommands+= "setReference $POSE$",
                           "setInitialReachDist 0.3",
                           "setStayWithinDist 0.4",
                           "setStayTime 0.4",

                                      "goto 0 0 0 15",
                                      "goto 0 0 -0.25 30",
                                      "goto 0 0 0.25 40",
                                      "goto 0 0 -0.25 30",
                                      "goto 0 0 0.25 15",
                                      "goto 0 0 -0.25 0",
                                        "goto 0 0 0 -15",
                                        "goto 0 0 -0.25 -30",
                                        "goto 0 0 0.25 -40",
                                        "goto 0 0 -0.25 -30",
                                        "goto 0 0 0.25 -15",
                                      "goto 0 0 0 0";

//    simpleExploreCommands+= "setReference $POSE$",
//                           "setInitialReachDist 0.3",
//                           "setStayWithinDist 0.4",
//                           "setStayTime 0.4",

//                                      "goto 0 0 0  20",
//                                      "goto 0 0 -0.15 30",
//                                      "goto 0 0 0.50 40",
//                                      "goto 0 0 -0.25 30",
//                                      "goto 0 0 0.25 15",
//                                      "goto 0 0 -0.25 0",
//                                        "goto 0 0 0 -15",
//                                        "goto 0 0 -0.25 -30",
//                                        "goto 0 0 0.25 -40",
//                                        "goto 0 0 -0.25 -30",
//                                        "goto 0 0 0.25 -15",
//                                      "goto 0 0 0 0";

    _360exploreCommands+= "setReference $POSE$",
                          "setInitialReachDist 0.3",
                          "setStayWithinDist 0.4",
                          "setStayTime 0.1",

                                       "goto 0.4 0 0 0       ",
                                       "goto -0.4 0 0 0      ",
                                       "goto 0 0 0.25 10     ",
                                       "goto 0 0 -0.25 20    ",
                                       "goto 0 0 0.25 30     ",
                                       "goto 0 0 -0.25 40    ",
                                       "goto 0 0 0.25 50     ",
                                       "goto 0 0 -0.25 60    ",
                                       "goto 0 0 0.25 70     ",
                                       "goto 0 0 -0.25 80    ",
                                       "goto 0 0 0.25 90     ",
                                       "goto 0 0.4 0.25 90   ",
                                       "goto 0 -0.4 0.25 90  ",
                                       "goto 0 0 -0.25 100   ",
                                       "goto 0 0 0.25 110    ",
                                       "goto 0 0 -0.25 120   ",
                                       "goto 0 0 0.25 130    ",
                                       "goto 0 0 -0.25 140   ",
                                       "goto 0 0 0.25 150    ",
                                       "goto 0 0 -0.25 160   ",
                                       "goto 0 0 0.25 170    ",
                                       "goto 0 0 -0.25 180   ",
                                       "goto 0.4 0 -0.25 180 ",
                                       "goto -0.4 0 -0.25 180",
                                       "goto 0 0 0.25 190    ",
                                       "goto 0 0 -0.25 200   ",
                                       "goto 0 0 0.25 210    ",
                                       "goto 0 0 -0.25 220   ",
                                       "goto 0 0 0.25 230    ",
                                       "goto 0 0 -0.25 240   ",
                                       "goto 0 0 0.25 250    ",
                                       "goto 0 0 -0.25 260   ",
                                       "goto 0 0 0.25 270    ",
                                       "goto 0 0.4 0.25 270  ",
                                       "goto 0 -0.4 0.25 270 ",
                                       "goto 0 0 -0.25 280   ",
                                       "goto 0 0 0.25 290    ",
                                       "goto 0 0 -0.25 300   ",
                                       "goto 0 0 0.25 310    ",
                                       "goto 0 0 -0.25 320   ",
                                       "goto 0 0 0.25 330    ",
                                       "goto 0 0 -0.25 340   ",
                                       "goto 0 0 0.25 350    ",
                                       "goto 0 0 -0.25 360   ",
                                       "goto 0 0 0 0         ",
                                       "goto 0 0 -0.25 10    ",
                                       "goto 0 0 0 20    ";



  }
void run(){



    std::cout << "Record Time: " << std::endl;

    cin >> RecTime;
    recordTime=ros::Duration(RecTime);    //record for 3 minutes
    ros::Rate r(10); // 10 hz
    std::cout << "Please enter exploration mode: " << std::endl;

    cin >> exploration_mode;
    switch(exploration_mode){
    case 0:
        selectedCommands=simpleExploreCommands;
        break;
     case 1 :
        selectedCommands=_360exploreCommands;
        break;
     default:
        selectedCommands=simpleExploreCommands;
        break;
    }

    r.sleep();
//    publishCommand(n,tum_ardrone_pub,n.resolveName("tum_ardrone/com"),"c clearCommands");
//    publishCommand(n,tum_ardrone_pub,n.resolveName("tum_ardrone/com"),"c discover1");
//    publishCommand(n,tum_ardrone_pub,n.resolveName("tum_ardrone/com"),"c start");
    ros::spinOnce();
    recordBegin = ros::Time::now();
    ros::Time recordEnd = ros::Time::now();

    undistortImage();
    ros::spinOnce();

//    rec1.sleep();
        noCommandsPublisher.publish(std_msgs::Empty());

    //mtmi()
    while(ros::ok() && (recordEnd-recordBegin)<recordTime){
      ros::spinOnce();
      r.sleep();
      recordEnd = ros::Time::now();
    }
    cout<<"record ended... "<<endl;
    subImages.shutdown();
    subFrames.shutdown();
    subKalmanFilter.shutdown();
    frameBag.close();
    KalmanFilterBag.close();
    //record is over

    match_ID=findMatch();
    queryPoints=getQueryCoordinates();
    match2dPoints=getMatchCoordinates();



    getMatchedFrame(match_ID);
    getPositionFromKalmanFilter();
    calculate3DPoints(match2dPoints,queryPoints,matchedFrame);




    //Key Frame position in world coordinate

    cout<<"MatchID is : "<<match_ID<<endl;
//    cout<<"size of 2d points: "<<match2dPoints.size()<<endl;
//    cout<<"size of 3d points: "<<match3dPoints.size()<<endl;
    int pointNumber=selected2dPoints.size();
    cout<<"size of selected 2d points: "<<pointNumber<<endl;

    //  SolvePnP
    cv::Mat K = (cv::Mat_<float>(3, 3) << matchedFrame->fx, 0, matchedFrame->cx, 0, matchedFrame->fy, matchedFrame->cy, 0, 0, 1);
    cv::Mat distortion = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

double solvePnPThreshold=6.0;
if(pointNumber>150)
    solvePnPThreshold=5.0;
if(pointNumber>250)
    solvePnPThreshold=4.0;
if(pointNumber>400)
    solvePnPThreshold=3.0;



//cv::solvePnPRansac(match3dPoints,selected2dPoints,K,distortion,rotation_vector,translation_vector);
    cout<<"solvepnp threshold is: "<<solvePnPThreshold<<endl;
    cv::solvePnPRansac(match3dPoints,selected2dPoints,K,distortion,rotation_vector,translation_vector, false, 100, 8.0, 100);



calculateCommand();

    cout << "Command is"<< endl;

//    snprintf(buf,200,"gotoraw %.3f %.3f %.3f %.3f", result[0],result[1],result[2],testYaw);
//    cout <<buf<<endl;


    char buf[200];
//    snprintf(buf,200,"gotoraw %.3f %.3f %.3f %.3f", result[0]/3,result[1]/3,result[2]/3,testYaw);
    ostringstream oss;
    oss << "c gotoraw " <<result[0]<<" "<<result[1]<<" "<<result[2]<<" "<<testYaw;
    finalCommand=oss.str();
    cout <<finalCommand<<endl;

    char buf2[200];
    snprintf(buf2,200,"gotoraw %.3f %.3f %.3f %.3f", 2*result[0]/3,2*result[1]/3,2*result[2]/3,testYaw);
    cout <<buf2<<endl;

    char buf3[200];
    snprintf(buf3,200,"gotoraw %.3f %.3f %.3f %.3f", result[0],result[1],result[2],testYaw);
    cout <<buf3<<endl;

    drone_state=READY_TO_GO;


    if(drone_prev_state==WAITING || drone_prev_state==EXPLORED)
    {
        cout<<"publishing nocommand. state: "<<drone_state<<std::endl;
        noCommandsPublisher.publish(std_msgs::Empty());
    }

    ros::spin();
}

};

int main(int argc, char **argv)
 {
    ros::init(argc, argv, "find_camera_pose");
    ros::NodeHandle n;

    find_camera_pose_node f(n);
    f.run();

  return 0;
}


