#include <fstream>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <yaml-cpp/yaml.h>


int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./bag_get_times_node config_file_path" << std::endl;

        return 1;
    }

    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = output_folder_path + bag_name;

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    
    // std::string l_cam = "/mynteye/left/image_raw";
    // std::string r_cam = "mynteye/right/image_raw";
    // std::string imu = "/mynteye/imu/data_raw";
    // std::string lidar = "/velodyne_points";

    std::string l_cam_topic = config_node["left_cam_topic"].as<std::string>();
    std::string r_cam_topic = config_node["right_cam_topic"].as<std::string>();
    std::string imu_topic = config_node["imu_topic"].as<std::string>();
    std::string lidar_topic = config_node["lidar_topic"].as<std::string>();

    // Image topics to load
    std::vector<std::string> topics;
    topics.push_back(l_cam_topic);
    topics.push_back(r_cam_topic);
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    std::cout << "ROS bag time: " << (bag_end_time - bag_begin_time).toSec() << "(s)" << std::endl; 

    const std::string file_name =  "header & time_stamp_left.txt";
    const std::string file_name_imu =  "header & time_stamp_imu.txt";
    const std::string file_name_lidar =  "header & time_stamp_lidar.txt";
    std::ofstream f;
    std::ofstream f_imu;
    std::ofstream f_lidar;
    f.open(file_name.c_str());
    f_imu.open(file_name_imu.c_str());
    f_lidar.open(file_name_lidar.c_str());
    // f << fixed;
    int times = 0;
    
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        // --------------------left-----------------------------------
        if (m.getTopic() == l_cam_topic || ("/" + m.getTopic() == l_cam_topic))
        {
            // std::cout << "in circle" << std::endl;
            std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
            ros::Time test_time = m.getTime();// use an instance to call getTime()
            long long test_time_value = test_time.toNSec();
            std::cout << "test time: " << test_time_value << std::endl;
            

            sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
            ros::Time header_time = l_img->header.stamp;
            long long header_time_value = header_time.toNSec();
            std::cout << "header time: " << header_time_value << std::endl;
            

            long long diff_time_value = test_time_value - header_time_value;
            std::cout << "diff time: " << diff_time_value << std::endl;

            f << test_time_value << "\t" <<  header_time_value << "\t" << diff_time_value << "\n";
            
            // if (times > 10){
            //     f.close();
            //     bag.close();
            //     return 0;
            // }
            // times++;
        }
        // ---------------------------imu----------------------------------------
        if (m.getTopic() == imu_topic || ("/" + m.getTopic() == imu_topic))
        {
            // std::cout << "in circle" << std::endl;
            std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
            ros::Time test_time = m.getTime();// use an instance to call getTime()
            long long test_time_value = test_time.toNSec();
            std::cout << "test time: " << test_time_value << std::endl;
            

            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            ros::Time header_time = imu->header.stamp;
            long long header_time_value = header_time.toNSec();
            std::cout << "header time: " << header_time_value << std::endl;
            

            long long diff_time_value = test_time_value - header_time_value;
            std::cout << "diff time: " << diff_time_value << std::endl;

            f_imu << test_time_value << "\t" <<  header_time_value << "\t" << diff_time_value << "\n";
            
            // if (times > 10){
            //     f.close();
            //     bag.close();
            //     return 0;
            // }
            // times++;
        }
        // -------------------------lidar-------------------------------
        if (m.getTopic() == lidar_topic || ("/" + m.getTopic() == lidar_topic))
        {
            // std::cout << "in circle" << std::endl;
            std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
            ros::Time test_time = m.getTime();// use an instance to call getTime()
            long long test_time_value = test_time.toNSec();
            std::cout << "test time: " << test_time_value << std::endl;
            

            sensor_msgs::PointCloud2::ConstPtr lidar = m.instantiate<sensor_msgs::PointCloud2>();
            ros::Time header_time = lidar->header.stamp;
            long long header_time_value = header_time.toNSec();
            std::cout << "header time: " << header_time_value << std::endl;
            

            long long diff_time_value = test_time_value - header_time_value;
            std::cout << "diff time: " << diff_time_value << std::endl;

            f_lidar << test_time_value << "\t" <<  header_time_value << "\t" << diff_time_value << "\n";
            
            // if (times > 10){
            //     f.close();
            //     bag.close();
            //     return 0;
            // }
            // times++;
        }
    }
    f.close();
    f_imu.close();
    f_lidar.close();
    bag.close();

    return 0;
}