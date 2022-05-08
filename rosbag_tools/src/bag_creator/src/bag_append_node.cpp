#include <iostream>
#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./bag_append_node config_file_path" << std::endl;

        return 1;
    }

    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    std::string output_bag_name = config_node["output_bag_name"].as<std::string>();
    std::string output_bag_path = output_folder_path + output_bag_name;

    rosbag::Bag bag;
    bag.open(output_bag_path, rosbag::bagmode::Append);// append topics to existing rosbag
    // bag.open("/home/Rossen/Datasets/MH_01_easy.bag", rosbag::bagmode::Append);

    std::string l_cam_topic = config_node["left_cam_topic"].as<std::string>();
    std::string r_cam_topic = config_node["right_cam_topic"].as<std::string>();

    // std::string l_cam_topic = "/dalsa_gray_0/image_raw";
    // std::string r_cam_topic = "/dalsa_gray_1/image_raw";
    // std::string r_cam = "/cam1/image_raw";

    std::string l_path = config_node["left_folder_path"].as<std::string>();
    std::string r_path = config_node["right_folder_path"].as<std::string>();
    
    std::vector<double> vTimeStamps;
    std::ifstream fTimes;
    std::string strPathTimes = config_node["timestamp_file_path"].as<std::string>();
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(72000);

    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    int flag = 0;
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            // vTimeStamps.push_back(t);
            if (flag % down_sample_ratio == 0){
                vTimeStamps.push_back(t);
            }
            flag++;  
        }   
    }

    const int nTimes = vTimeStamps.size();

    // for (size_t i = 0; i < nTimes; i++)
    for (size_t i = 0; i < nTimes; i++)
    {
        // 1:left  2:right
        
        std::string file_name = config_node["left_cam_prefix"].as<std::string>();
        std::string file_name2 = config_node["right_cam_prefix"].as<std::string>();
        int batch = i*down_sample_ratio/3000;
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i*down_sample_ratio+1;
        std::stringstream sss;
        sss << std::setfill('0') << std::setw(6) << batch;
        file_name = file_name + sss.str() + "_" + ss.str() + ".tif";
        file_name2 = file_name2 + sss.str() + "_" + ss.str() + ".tif";

        int imread_flag = config_node["imread_flag"].as<int>();// imread enum flag: 0=grayscale, 1=color
        cv::Mat img = cv::imread(l_path + file_name, imread_flag);
        if (img.empty()){
            std::cout << "failed to read: " << l_path << file_name << std::endl;
            std::cout << "SHUT DOWN!" << std::endl;
            return 0;
        }
        cv::Mat img2 = cv::imread(r_path + file_name2, imread_flag);
        if (img2.empty()){
            std::cout << "failed to read: " << r_path <<file_name2 << std::endl;
            std::cout << "SHUT DOWN!" << std::endl;
            return 0;
        }
        // cv_bridge::CvImage ros_image;
        // sensor_msgs::ImagePtr ros_image_msg;
        // ros_image.image = img;
        // ros_image.encoding = "mono8";
        // ros_image_msg = ros_image.toImageMsg();

        std::string image_format = config_node["image_format"].as<std::string>();

        sensor_msgs::ImagePtr ros_image_msg = cv_bridge::CvImage(std_msgs::Header(), image_format, img).toImageMsg();
        sensor_msgs::ImagePtr ros_image_msg2 = cv_bridge::CvImage(std_msgs::Header(), image_format, img2).toImageMsg();

        ros_image_msg->header.stamp = ros::Time().fromSec(vTimeStamps[i]);
        ros_image_msg->header.frame_id = l_cam_topic;
        bag.write(l_cam_topic, ros::Time().fromSec(vTimeStamps[i]), ros_image_msg);

        ros_image_msg2->header.stamp = ros::Time().fromSec(vTimeStamps[i]);
        ros_image_msg2->header.frame_id = r_cam_topic;
        bag.write(r_cam_topic, ros::Time().fromSec(vTimeStamps[i]), ros_image_msg2);
        std::cout << "wrote frame: " << i+1 << std::endl;
    }

    std::cout << "---------end---------" << std::endl;
    
    bag.close();

    return 0;
}