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
        std::cerr << std::endl << "Usage: ./bag_extractor_node config_file_path" << std::endl;

        return 1;
    }

    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = output_folder_path + bag_name;

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::string l_cam_topic = config_node["left_cam_topic"].as<std::string>();
    std::string r_cam_topic = config_node["right_cam_topic"].as<std::string>();

    // Image topics to load
    std::vector<std::string> topics;
    topics.push_back(l_cam_topic);
    topics.push_back(r_cam_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // ros::Time bag_begin_time = view.getBeginTime();
    // ros::Time bag_end_time = view.getEndTime();

    // std::cout << "ROS bag time: " << (bag_end_time-bag_begin_time).toSec() << "(s)" << std::endl;

    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        if (m.getTopic() == l_cam_topic || ("/" + m.getTopic() == l_cam_topic))
        {
            sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
            if (l_img != NULL){
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(l_img, l_img->encoding);
                cv::imwrite(output_folder_path + "left/" + std::to_string(l_img->header.stamp.toNSec()) + ".png", cv_ptr->image);// path must exist
                
                // for test
                // bag.close();
                // return 0;
            }
        }

        if (m.getTopic() == r_cam_topic || ("/" + m.getTopic() == r_cam_topic))
        {
            sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
            if (r_img != NULL){
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(r_img, r_img->encoding);
                cv::imwrite(output_folder_path + "right/" + std::to_string(r_img->header.stamp.toNSec()) + ".png", cv_ptr->image);// path must exist
                
                // for test
                // bag.close();
                // return 0;
            }
        }
    }
    
    bag.close();

    return 0;
}