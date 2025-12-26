//
// Created by phil on 24/01/18.
// Updated by Saad on 11/12/25 to ROS2. 
//

// #include <gazebo/transport/transport.hh>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/gazebo_client.hh>
// #include <gazebo/gazebo_config.h> // remove?

#include <nav_msgs/msg/odometry.hpp> //<nav_msgs/Odometry.h>
#include <rclcpp/rclcpp.hpp> // <ros/ros.h>
// #include <geometry_msgs/Vector3.h> // remove?
#include <std_msgs/msg/bool.hpp> // <std_msgs/Bool.h>
#include <ros_gz_interfaces/msg/contacts.hpp>

#include <string>

// #include <iostream>
// #include <vector>

// using gazebo::msgs::Contacts;
// using gazebo::msgs::ContactsPtr;
const std::string DELIMITER = "::";
using std::placeholders::_1;

class ForceMeasureNode : public rclcpp::Node {
public:
    ForceMeasureNode()
    : Node("force_measure"), airborne_(false)
    {
        collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision", 1000);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth/state", 
            1000, 
            std::bind(&ForceMeasureNode::positionCb, this, _1));
        
        contacts_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/touched",
            1000,
            std::bind(&ForceMeasureNode::forcesCb, this, _1));

    }

private:
    bool airborne_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_sub_;

    void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg){
        airborne_ = (msg->pose.pose.position.z > 0.3);
    }

    void forcesCb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg){
        for(const auto &contact : msg -> contacts){
            std::string entity1 = contact.collision1.name;
            entity1 = entity1.substr(0, entity1.find(DELIMITER)); // Extract entity1 name

            std::string entity2 = contact.collision2.name;
            entity2 = entity2.substr(0, entity2.find(DELIMITER)); // Extract entity2 name

            if(entity1 != "ground_plane" && entity2 != "ground_plane"){
                if(entity1 == "jackal/robot" || entity2 == "jackal/robot"){
                    std_msgs::msg::Bool collide;
                    collide.data = true;
                    collision_pub_->publish(collide);
                    RCLCPP_INFO(this->get_logger(), "%s : %s", entity1.c_str(), entity2.c_str());
                    return;
                }
            }

        }
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceMeasureNode>());
    rclcpp::shutdown();
    return 0;
}

// auto pub; //ros::Publisher pub;

// bool airborne;


// // Forces callback function
// void forcesCb(ConstContactsPtr &_msg){
//     // What to do when callback
//     for (int i = 0; i < _msg->contact_size(); ++i) {

//         std::string entity1 = _msg->contact(i).collision1();
//         entity1 = entity1.substr(0, entity1.find(DELIMITER)); // Extract entity1 name

//         std::string entity2 = _msg->contact(i).collision2();
//         entity2 = entity2.substr(0, entity2.find(DELIMITER)); // Extract entity1 name

//         if(entity1 != "ground_plane" && entity2 != "ground_plane"){
//             if (entity1 == "jackal" || entity2 == "jackal"){
//                 std_msgs::Bool collide;
//                 collide.data = true;
//                 pub.publish(collide);
//                 ROS_INFO_STREAM(entity1 + ":" + entity2);
//                 return;
//             }
//         }
//     }
// }

// // Position callback function
// void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg2){
//     if (msg2->pose.pose.position.z > 0.3) {
//         airborne = true;
//     } else {
//         airborne = false;
//     }
// }

// int main(int _argc, char **_argv){
//     // Set variables
//     // airborne = false;

//     // Load Gazebo & ROS
//     gazebo::client::setup(_argc, _argv);
//     // rclcpp::init(_argc, _argv); // ros::init(_argc, _argv, "force_measure"); done by super call 

//     // Create Gazebo node and init
//     gazebo::transport::NodePtr node(new gazebo::transport::Node());
//     node->Init();

//     // Create ROS node and init
//     // auto n = rclcpp::Node::make_shared("force_measure"); // ros::NodeHandle n; done
//     // pub = n->create_publisher<std_msgs::msg::Bool>("collision", 1000); //pub = n.advertise<std_msgs::Bool>("collision", 1000); done

//     // Listen to Gazebo contacts topic
//     gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", forcesCb);

//     // Listen to ROS for position
//     // auto sub2 = n->create_subscription<nav_msgs::msg::Odometry>("ground_truth/state", 1000, positionCb); //ros::Subscriber sub2 = n.subscribe("ground_truth/state", 1000, positionCb); done

//     // Busy wait loop...replace with your own code as needed.
//     while (true)
//     {
//         gazebo::common::Time::MSleep(20);

//         // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
//         rclcpp::spin_all(n, 0s); // ros::spinOnce();


//     // Mayke sure to shut everything down.

//     }
//     gazebo::client::shutdown();
// }