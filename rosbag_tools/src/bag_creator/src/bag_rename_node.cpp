#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
    
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: rosrun bag_creator bag_rename_node config_file_path" << std::endl;

        return 1;
    }

    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    
    std::string old_topic = config_node["old_topic"].as<std::string>();
    std::string new_topic = config_node["new_topic"].as<std::string>();
    std::string old_bag = config_node["old_bag"].as<std::string>();
    std::string new_bag = config_node["new_bag"].as<std::string>();
    rosbag::Bag bag_read;
    rosbag::Bag bag_write;
    bag_read.open(old_bag, rosbag::bagmode::Read);
    bag_write.open(new_bag, rosbag::bagmode::Write);
    rosbag::View view_all(bag_read);
    int flag = 0;
    for(const rosbag::MessageInstance& m : view_all) {
        if (m.getTopic() == old_topic) {
            std::cout << m.getTopic() << std::endl;
            bag_write.write(new_topic, m.getTime(), m);
            std::cout << "-----------" << ++flag << "-----------" << std::endl;
        } else {
            bag_write.write(m.getTopic(), m.getTime(), m);
        }
    }
    bag_read.close();
    bag_write.close();
    return 0;
}