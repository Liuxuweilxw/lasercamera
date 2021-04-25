#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "opencv/cxeigen.hpp"

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"

cv::Mat cv_T_;
CameraPtr cameraptr_;
namespace sc
{

    void TranScanToPoints(const sensor_msgs::LaserScanConstPtr& scan_in, std::vector<Eigen::Vector3d>& Points)
    {
        // http://wiki.ros.org/laser_geometry
        size_t n_pts = scan_in->ranges.size ();
        Eigen::ArrayXXd ranges (n_pts, 2);
        Eigen::ArrayXXd output (n_pts, 2);
        Eigen::ArrayXXd co_sine_map (n_pts, 2);

//        std::cout << "---------- read scan -----------"<<std::endl;
        for (size_t i = 0; i < n_pts; ++i)
        {
            // std::cout << scan_in->ranges[i]<< " "<<scan_in->angle_min + (double) i * scan_in->angle_increment <<std::endl;
            ranges (i, 0) = (double) scan_in->ranges[i];
            ranges (i, 1) = (double) scan_in->ranges[i];

            co_sine_map (i, 0) = cos (scan_in->angle_min + (double) i * scan_in->angle_increment);
            co_sine_map (i, 1) = sin (scan_in->angle_min + (double) i * scan_in->angle_increment);
        }

        output = ranges * co_sine_map;

        for (size_t i = 0; i < n_pts; ++i) {

            // TODO: range_cutoff threashold
            double range_cutoff = 30.0;
            const float range = scan_in->ranges[i];
            if (range < range_cutoff && range >= scan_in->range_min)
            // if (range < range_cutoff && range >= 0.5)
            {
                Points.push_back(Eigen::Vector3d(output (i, 0), output (i, 1), 0) );
            }
            else
            {
                Points.push_back(Eigen::Vector3d(1000.0,1000.0, 0) );
            }
        }
       

    }

    void showScanCallback(const sensor_msgs::LaserScanConstPtr &scan_in, 
                          const sensor_msgs::ImageConstPtr &image_in) {

        std::vector<Eigen::Vector3d> p_l;
        TranScanToPoints(scan_in, p_l);
        std::cout<<"scan_in's size : "<<scan_in->ranges.size()<<std::endl;
        std::cout<<"p_l's size : "<<p_l.size()<<std::endl;

        Eigen::Matrix4d Tlc;
        Tlc.setZero();

        if(!cv_T_.empty())
            cv::cv2eigen(cv_T_, Tlc);   //Tlc 4x4 matrix
        else{
            std::cerr <<" You Do not have calibra result Tlc. We use a Identity matrix." << std::endl;
            Tlc.setIdentity();
        }
        // std::cout<<"Tlc : "<<Tlc<<std::endl;

        Eigen::Matrix3d Rlc = Tlc.block(0,0,3,3); 
        Eigen::Vector3d tlc(Tlc(0,3),Tlc(1,3),Tlc(2,3));

        Eigen::Matrix3d Rcl = Rlc.transpose();
        Eigen::Vector3d tcl = - Rcl * tlc;

        std::cout<<"Rcl :"<<Rcl<<std::endl;
        std::cout<<"tcl : "<<tcl<<std::endl;

//        std::cout << " Tlc: \n"<< Tlc<<std::endl;

        std::vector<Eigen::Vector3d> p_c;
        std::vector<cv::KeyPoint> keypoints;

        int n_pts = scan_in->ranges.size();
        // std::cout<<"n_pts : "<<n_pts<<std::endl;
        for (int i = 0; i < n_pts; i++) {
            // if(p_l[i].x()>0) continue;
           // std::cout << p_l[i].transpose() << std::endl;
            p_c.emplace_back(Rcl * p_l[i] + tcl); 
            std::cout<<"p_l["<<i<<"] : "<<std::endl<<p_l[i].x()<<"   "<<p_l[i].y()<<"  "<<p_l[i].z()<<std::endl;
            // std::cout<<"p_c["<<i<<"] : "<<std::endl<<p_c[i].x()<<"   "<<p_c[i].y()<<"  "<<p_c[i].z()<<std::endl;
            std::cout<<"p_c : "<<Rcl * p_l[i] + tcl<<std::endl;
        }

        std::vector< cv::Point2f > pixel;
        std::vector< cv::Point3f> pts;

        n_pts = p_c.size();

        for (int i = 0; i < n_pts; i++) {       

            double X = p_c[i].x();
            double Y = p_c[i].y();
            double Z = p_c[i].z();

             if(Z<0) continue;

            // std::cout<<"X : "<<X<<std::endl;
            // std::cout<<"Y : "<<Y<<std::endl;
            // std::cout<<"Z : "<<Z<<std::endl;

            ////
            // if(X<0){
            pts.push_back( cv::Point3f(X,Y,Z) );
            // }
           

        }
        cv::Mat r = cv::Mat::zeros(3,1,CV_64F);
        // double rm[3][1]={-2.6529, -0.7876, -0.7265};
        // double rm[3][1]={-2.6529536542531802e+00, -7.8763724779270472e-01,-7.2655337657740171e-01};
        // cv::Mat r = cv::Mat(3,1,CV_64F, rm);
        // std::cout<<"r : "<<r<<std::endl;
        cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
        // double tm[3][1] = {0.4971, 0.1141, 0.5984};
        // double tm[3][1] = {4.9709145127037113e-01, 1.1414970688410903e-01, 5.9839525355093004e-01};
        // cv::Mat t = cv::Mat(3,1,CV_64F, tm);
        // std::cout<<"t : "<<t<<std::endl;
        std::cout<<"pts's size : "<<pts.size()<<std::endl;
        cameraptr_->projectPoints(pts, r, t,pixel);
        std::cout<<"pixel's size : "<<pixel.size()<<std::endl;
//        cv::projectPoints(pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), cvK_, cvD_, pixel);

        cv::Mat img_src = cv_bridge::toCvShare(image_in, "bgr8")->image;

        for (size_t j = 0; j < pixel.size(); ++j) {
            // cv::circle(img_src, pixel[j],1, cv::Scalar(0,255,0),1);
            // std::cout<<pixel[j]<<std::endl;
            cv::circle(img_src, pixel[j],3, cv::Scalar(0,0,255),-1);
        }
        cv::imshow("show", img_src);

        cv::waitKey(1);
    }

} // sc namespace end

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    std::cout << name <<std::endl;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_scan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /// ==========================================
    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return 0;
    } else
    {
        std::cout << "READ CONFIG_FILE...\n";
    }

    std::string scan_topic_name;
    std::string img_topic_name;
    // std::string bag_path;
    std::string save_path;

    fsSettings["scan_topic_name"] >> scan_topic_name;
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["savePath"] >> save_path;
    // fsSettings["bag_path"] >> bag_path;


    std::string sModelType;
    fsSettings["model_type"] >> sModelType;

    cameraptr_ = nullptr;
    if (sModelType.compare("KANNALA_BRANDT") == 0)
    {
        EquidistantCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new EquidistantCamera(paras));
        ROS_INFO("LOAD KANNALA BRANDT CAMERA!");
    }
        // todo:: PIN HOLE MODEL
    else
    {
        PinholeCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new PinholeCamera(paras));
    }


    fsSettings.release();

    std::cout <<"\t"<<scan_topic_name << std::endl;
    std::cout <<"\t"<<img_topic_name << std::endl;
    // std::cout <<"\t"<<bag_path << std::endl;

    std::string fn = save_path + "result.yaml";
    fsSettings.open(fn,cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "You should run calibra tool first to generate result.yaml in your savePath." << std::endl;
        return 0;
    }
    fsSettings["extrinsicTlc"] >> cv_T_;
    std::cout <<"Load extrinsicTlc:\n"<<cv_T_<<std::endl;
    fsSettings.release();

    std::cout << "END CONFIG_FILE\n" << std::endl;
/// ==========================================

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, scan_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, img_topic_name, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), scan_sub, image_sub);
    sync.registerCallback(boost::bind(&sc::showScanCallback, _1, _2));
    std::cout << "start spin.." << std::endl;
    ros::spin();

    return 0;
}
