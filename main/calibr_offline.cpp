#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include<string>

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"
#include "config.h"
#include "utilities.h"
#include "selectScanPoints.h"
#include "calcCamPose.h"
#include "LaseCamCalCeres.h"
#include "time.h"
#include<fstream>
#include<iostream>
#include<algorithm>
// #include<fstream>
#include<chrono>
#include<unistd.h>
// #include<string>
#include<vector>
#include<iomanip>
#include"stdlib.h"

#include <algorithm>

using namespace std;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  std::cout << name <<std::endl;
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

bool compare_points(Eigen::Vector3d pointA, Eigen::Vector3d pointB){
  return  pointA.x()<pointB.x();
}

int main(int argc, char **argv){
  string outputPath =  "/home/liuxuwei/Files/laser_data.txt";
	ofstream file_output;
	file_output.open(outputPath.c_str());

  ros::init(argc, argv, "LaserCamCal");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_file;
  config_file = readParam<std::string>(pnh, "config_file");
  readParameters(config_file);

  rosbag::Bag bag_input;
  bag_input.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(scan_topic_name);
  topics.push_back(img_topic_name);
  rosbag::View views(bag_input, rosbag::TopicQuery(topics));

  // Load apritag pose
  std::vector < CamPose > tagpose;
  LoadCamPoseFromTxt(savePath + "apriltag_pose.txt",tagpose);
  std::cout << "Load apriltag pose size: " << tagpose.size() <<std::endl;

  if(tagpose.size() < 10)
  {
    std::cout << "apriltag pose less than 10." << std::endl;
    return 0;
  }

  /// Select keyframe to calibrating
  std::vector< CamPose > sparseTagpose;
  CamPose older = tagpose.at(0);
  sparseTagpose.push_back(older);
  double dist_min = 0.20;  //10cm? not 20cm?
  double theta_min = 3.1415926 * 10/ 180.;
  // std::cout<<"Theta_min"<<theta_min<<std::endl;//pass
  for (int j = 1; j < tagpose.size(); ++j) {
    // std::cout<<"The echo is :"<<j<<std::endl;
    CamPose newer = tagpose.at(j);
    double dist = (older.twc - newer.twc).norm();
    double theta = 2 * std::acos( (older.qwc.inverse() * newer.qwc).w() );
    if( (dist > dist_min) || (fabs(theta) > theta_min) )
    {
      older = newer;
      sparseTagpose.push_back(older);
    }
  }
  // std::cout << "Tagpose's size is: " << tagpose.size() <<std::endl;
  // std::cout<<"sparseTagpose's size is :"<<sparseTagpose.size()<<std::endl;
  tagpose = sparseTagpose;

  // 准备标定数据
  std::vector<Oberserve> obs;

  // 处理激光数据
  int ii_cnt = 10;

int count=0;
int echo = 0;
  for(rosbag::MessageInstance const m: views){
    
    if (m.getTopic() == scan_topic_name)
    {
      sensor_msgs::LaserScan::Ptr scan = m.instantiate<sensor_msgs::LaserScan>();
      std::vector<Eigen::Vector3d> Points;


      
      TranScanToPoints(*scan,Points);
      // std::cout<<"Points 's size : "<<Points.size()<<std::endl;



    


//      ii_cnt++;
//      if(ii_cnt % 20 != 0) continue;
      
      double timestamp = scan->header.stamp.toSec();
      std::vector<Eigen::Vector3d> points;
      std::cout<<"Echo :" <<echo++<<std::endl;
      points = AutoGetLinePts(Points);
      
      // for(int i=0;i<points.size();i++){
      //   for(int j=0;j<3;j++){
      //     file_output<<setw(10)<<setfill(' ')<<setiosflags(ios::fixed)<<setprecision(2)<<points.at(i)[j];
      //   }
      //   file_output<<endl;
      //   // std::cout<<Points.at(i)<<" ";
      // }
      

      // sleep(1);
      // 检测到了直线
      
      // std::cout<<"Line is detected ? : "<<points.size()<<std::endl;
      // std::cout<<"Points's size "<<points.size()<<std::endl;
      if(points.size() > 0)
      {
        // 在 camera 里找时间戳最近的一个 pose
        double min_dt = 10000;
        CamPose colsetTagPose;
        for (int i = 0; i < tagpose.size(); ++i) 
        {
          CamPose tmp = tagpose.at(i);
          double t = fabs(tmp.timestamp - timestamp);
          if(t < min_dt)
          {
            min_dt = t;
            colsetTagPose = tmp;
          }
        }
        // std::cout<< "min_dt : " << min_dt <<std::endl;
        
        
        if(min_dt < 0.02)  // 20ms
        {



//          std::cout << "scan and tag time: "<<std::fixed<<std::setprecision(18)
//                    <<timestamp<<" "<<colsetTagPose.timestamp<<std::endl;
          /////////////////////////////////////////////////

          Eigen::Vector2d line;
          LineFittingCeres(points,line);

          std::cout<<"line0 : "<<line(0)<<std::endl;
          std::cout<<"line1 : "<<line(1)<<std::endl;

          // sleep(1);                 /////// ????


          sort(points.begin(),points.end(),compare_points);


          
          
          std::vector<Eigen::Vector3d> points_on_line;

          // 激光所在直线不能垂直于某个轴
          // double x_start(points.begin()->x()), x_end((points.end()-1)->x());
          // double y_start(points.begin()->y()), y_end((points.end()-1)->y());
          double x_start(points.at(0)[0]), x_end(points.at(points.size()-1)[0]);
          double y_start(points.at(0)[1]), y_end(points.at(points.size()-1)[1]);

          // std::cout<<"line0 : "<<line(0)<<std::endl;
          // std::cout<<"line1 : "<<line(1)<<std::endl;
          


          std::cout<<"x_start :"<<x_start<<std::endl;
          std::cout<<"y_start :"<<y_start<<std::endl;
          std::cout<<"x_end :"<<x_end<<std::endl;
          std::cout<<"y_end :"<<y_end<<std::endl;
          
          if( fabs(x_end - x_start) > fabs(y_end - y_start) )
          {
            y_start = - (x_start * line(0) + 1) / line(1);
            y_end = - (x_end * line(0) + 1) / line(1);

          } else // 可能垂直于 x 轴，采用y值来计算 x
          {
            x_start = - (y_start * line(1) + 1) / line(0);
            x_end = - (y_end * line(1) + 1) / line(0);
          }

          // std::cout<<"x_start222 :"<<x_start<<std::endl;
          // std::cout<<"y_start222 :"<<y_start<<std::endl;
          // std::cout<<"x_end222 :"<<x_end<<std::endl;
          // std::cout<<"y_end222 :"<<y_end<<std::endl;

          points_on_line.push_back(Eigen::Vector3d(x_start,y_start,0));
          points_on_line.push_back(Eigen::Vector3d(x_end,y_end,0));

          // if(count++%4==0)
          // {
            Oberserve ob;
            ob.tagPose_Qca = colsetTagPose.qwc.inverse();
            ob.tagPose_tca = -ob.tagPose_Qca.toRotationMatrix() * colsetTagPose.twc;
            ob.points = points;
            ob.points_on_line = points_on_line;
            std::cout<< "ready"<<std::endl;
            std::cout<<"points_on_line 's size :"<<points_on_line.size()<<std::endl;
            obs.push_back(ob);
            std::cout<<"Count : "<<count<<endl<<"start : "<<ob.points_on_line.at(0)[0]<<"  "
                                                                                                            << ob.points_on_line.at(0)[1]<<"  ," 
                                                                                        <<" end : "<<ob.points_on_line.at(1)[0]<<"  "
                                                                                                              <<ob.points_on_line.at(1)[1]<<std::endl;
            std::cout<<"tagPose_tca : "<<ob.tagPose_tca(0)<<"  "<<ob.tagPose_tca(1)<<"  "<<ob.tagPose_tca(2)<<"  "<<std::endl;
            std::cout<<"-------------------points data------------------- "<<std::endl;

            file_output<<"Count : "<<count<<endl;
            std::cout<<"points's size : "<<points.size()<<std::endl;
            for(int i=0;i<points.size();i++){
              for(int j=0;j<3;j++){
                std::cout<<setw(10)<<setfill(' ')<<setiosflags(ios::fixed)<<setprecision(2)<<points.at(i)[j];
                file_output<<setw(10)<<setfill(' ')<<setiosflags(ios::fixed)<<setprecision(2)<<points.at(i)[j];
            }
                std::cout<<endl;
                file_output<<endl;
                
            }
            count++;
              // std::cout<< "add success"<<std::endl;
            file_output<<"----------------------------------------"<<endl;
            // }
         }

          

      }
      

    }
    
  }


  std::cout <<"obs size: "<< obs.size() <<std::endl;
  
  if(obs.size() < 5)
  {
    std::cout << obs.size() << std::endl;
    std::cout << "Valid Calibra Data Less"<<std::endl;
    bag_input.close();
    return 0;
  }
  
////////////////////////////////////////////////

  Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
  CamLaserCalClosedSolution(obs,Tlc_initial);

  Eigen::Matrix4d Tcl = Tlc_initial.inverse();
  CamLaserCalibration(obs,Tcl, false);

////////////////////////////////////////////////////

  // // CamLaserCalibration(obs,Tcl, true);


///////////////////////////////////////////////////////////


  // Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();
  // CamLaserCalibration1(obs,Tcl,true);


// ///////////////////////////////////////////////////


  std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n"<<std::endl;
  Eigen::Matrix4d Tlc = Tcl.inverse();
  std::cout<< Tlc <<std::endl;

  std::cout << "\n----- Transform from Camera to Laser, euler angles and translations are: -----\n"<<std::endl;
  Eigen::Matrix3d Rlc(Tlc.block(0,0,3,3));
  Eigen::Vector3d tlc(Tlc.block(0,3,3,1));
  EulerAngles rpy =  ToEulerAngles(Eigen::Quaterniond(Rlc));
  std::cout << "   roll(rad): "<<rpy.roll <<" pitch(rad): "<<rpy.pitch << " yaw(rad): "<<rpy.yaw<<"\n"
            << "or roll(deg): "<<rpy.roll * 180./M_PI <<" pitch(deg): "<<rpy.pitch* 180./M_PI  << " yaw(deg): "<<rpy.yaw * 180./M_PI <<"\n"
            << "       tx(m): "<<tlc.x() << "  ty(m): "<<tlc.y() << "   tz(m): "<<tlc.z()<<std::endl;

  // save to yaml file
  cv::Mat cvTlc;
  cv::eigen2cv(Tlc,cvTlc);
  std::string fn = savePath + "result.yaml";
  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  fs << "extrinsicTlc"<<cvTlc;
  cv::Mat cvrpy;
  cv::eigen2cv(Eigen::Vector3d(rpy.roll,rpy.pitch,rpy.yaw),cvrpy);
  cv::Mat cvtlc;
  cv::eigen2cv(tlc,cvtlc);
  fs << "RollPitchYaw"<<cvrpy;
  fs << "txtytz"<<cvtlc;
  fs.release();


  std::cout << "\n Result file : "<<fn<<std::endl;
  std::cout << "\n-------------- Calibration Code End --------------\n"<<std::endl;

  file_output.close();
  ros::spin();
}
