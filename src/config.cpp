//
// Created by heyijia on 18-12-13.
//

#include "config.h"
#include <opencv2/opencv.hpp>

std::string savePath = "/home/liuxuwei/Files/mrobot_ws/src/log/";
std::string bag_path = "/home/liuxuwei/2021-04-06-21-12-55.bag";

std::string scan_topic_name = "/scan";
std::string img_topic_name = "/camera/color/image_raw";


void readParameters(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fsSettings["scan_topic_name"] >> scan_topic_name;
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["savePath"] >> savePath;
    fsSettings["bag_path"] >> bag_path;

    fsSettings.release();
}