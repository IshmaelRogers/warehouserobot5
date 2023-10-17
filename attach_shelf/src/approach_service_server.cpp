#include "checkpoint5_interfaces/srv/detail/go_to_loading__struct.hpp"
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

//create serviceServer class
class serviceServer : public rclcpp::Node
{
    public:
    serviceServer() : Node("service_server")
    {
        //declare and initialize a parameter name final_approach
        this->declare_parameter("final_approach", false);
        // Retrieve the value of the "final_approach" parameter and store it in the final_approach_ member variable
        this->get_parameter("final_approach", final_approach_);
        //create a service named "service" with the callback function "callback"
        srv_ = this->create_service<checkpoint5_interfaces::srv::GoToLoading>("/approach_shelf", std::bind(&serviceServer::handleService, this, std::placeholders::_1, std::placeholders::_2));
         // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        //create a tfBuffer
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        //create a tfListener
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
        //create a subscriber to subscribe to the /scan topic
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&serviceServer::laserCallback, this, _1));
        //create a publisher to publish to the cmd_vel topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        

    }

    private:
    //callback function
    void handleService(const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Response> response)
    {
    
        //print service request to the terminal
        RCLCPP_INFO(this->get_logger(), "Service Request Received");

        // detect laser intensities and find the midpoint between the legs
        findMidpoint();
        //print Finding midpoint complete 
        RCLCPP_INFO(this->get_logger(), "Finding midpoint complete");

        //handle service data
        if(request->attach_to_shelf == true && final_approach_ == true)
        {   
            //publish the cart TF
            createTF();
            //call moveToTF function
            moveToTF();
            //call moveForward function with a distance as input
            moveForward(0.3);
            //print calling liftShelf to the terminal
            RCLCPP_INFO(this->get_logger(), "Calling liftShelf function: ");
            //call liftShelf function
            //liftShelf();
            //if bothLegsDetected_ is true
            if(bothLegsDetected_)
            {
                //set response to true
                response->complete = true;
            }
            else
            {
                //set response to false
                response->complete = false;
            }
        }
        else if (request->attach_to_shelf == false || final_approach_ == false)
        {
            //set response to false
            response->complete = false;

    
        }
    }

    //findMidPoint function definition
    void findMidpoint()
    {
        //intizalize a vector to store the laser intensities
        std::vector<float> laser_intensities;
        //intizalize a vector to store laser ranges
        std::vector<float> laser_ranges; 
        //for loop through intensities_ to save all that are greater than 7000 to laser_intensities and the ranges at that intensity
        
        for(uint32_t i = 0; i < intensities_.size(); i++)
        {
            if(intensities_[i] == 8000)
            {   // find the range at that intensity
                float range = ranges_[i];
                laser_intensities.push_back(intensities_[i]);
                laser_ranges.push_back(range);
            }
        }
        //if the size of laser_intensities is less than 2 then bothLegsDetected_ is false and print a message to the terminal
        if(laser_intensities.size() < 2)
        {
            bothLegsDetected_ = false;
            RCLCPP_INFO(this->get_logger(), "Both legs not detected");
        }
        // if the size of laser_intensities is greater than 1 then bothLegsDetected_ is true and print a message to the terminal
        else if(laser_intensities.size() > 1)
        {
            bothLegsDetected_ = true;
            RCLCPP_INFO(this->get_logger(), "Both legs detected");
            //get the angle increment
            float theta = laser_msg_->angle_increment;
            //set leg1 to the first element of laser_ranges
            leg1 = laser_ranges[0];
            //set leg2 to the second element of laser_ranges
            leg2 = laser_ranges[1];
            //calculate the x position of leg1
            float x1 = leg1 * cos(theta);
            //calculate the x position of leg2
            float x2 = leg2 * cos(theta);
            //calculate y position of leg1
            float y1 = leg1 * sin(theta);
            //calculate y position of leg2
            float y2 = leg2 * sin(theta);
            //calculate the midpoint
            midpoint_ = (x1 + x2) / 2.0;
            //print calling createTF to the terminal
            RCLCPP_INFO(this->get_logger(), "Midpoint found! : ");     
        }
         
    }

    void createTF(){
        //create a TransformStamped message called t
        geometry_msgs::msg::TransformStamped t;
        //set header stamp to now
        t.header.stamp = this->now();
        //set header frame_id to robot_base_link
        t.header.frame_id = "robot_base_link";
        //set child_frame_id to cart_frame
        t.child_frame_id = "cart_frame";
        //set transform translation x to midpoint_
        t.transform.translation.x = midpoint_;
        //set transform translation y to 0.0 //REQUIRED
        t.transform.translation.y = 0.0;
        //set transform translation z to 0.0
        t.transform.translation.z = 0.0;
        //send the transform
        tf_broadcaster_->sendTransform(t);
        //print TF created to the terminal
        RCLCPP_INFO(this->get_logger(), "TF created");
        //print calling moveToTF to the terminal
        RCLCPP_INFO(this->get_logger(), "Calling moveToTF function: ");
        //call moveToTF function
        moveToTF();
    }

    //listener function
    void moveToTF(){
        //store frame names in variables to be use to compute transformations
        std::string target_frame = "cart_frame";
        std::string source_frame = "robot_base_link";
        //create a TransformStamped message called transformStamped
        geometry_msgs::msg::TransformStamped t2;
        //look up for the transformation between target_frame and source_frame
        try{

            t2 = tfBuffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

        } catch(tf2::TransformException &ex){
            //print the exception could not transform source to target frame
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            //return
            return;
        }
        // twist message
        geometry_msgs::msg::Twist twist_msg;
        static const double scaleRotationRate = 0.5;
        twist_msg.angular.z = scaleRotationRate * atan2(t2.transform.translation.y, t2.transform.translation.x);


        static const double scaleTranslationRate = 0.5;
        twist_msg.linear.x = scaleTranslationRate * sqrt(pow(t2.transform.translation.x, 2) + pow(t2.transform.translation.y, 2));

        //publish the twist message
        cmd_vel_publisher_->publish(twist_msg);

        }


    void moveForward(float distance){
        distance = 0.3;
    }

    //void liftShelf();

   void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // store the laser message
    laser_msg_ = scan;
    // store the laser intensities
    intensities_ = scan->intensities;
    // store the laser ranges
    ranges_ = scan->ranges;
    
  }

    // tf broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // tf buffer
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};
    // tf listener
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    rclcpp::Service<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr srv_;
    //vector to store range of the laser
    std::vector<float> ranges_;
    //vector to store intensities
    std::vector<float> intensities_;
    float midpoint_;
    bool bothLegsDetected_ = false;
    float leg1;
    float leg2;
    //laser subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    //
    //laser message
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;
    //twist message
    geometry_msgs::msg::Twist twist_msg;
    //cmd_vel publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    bool final_approach_;


};

//main function
int main(int argc, char **argv)
{
    //initialize ros
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<serviceServer>();
    //spin the node
    rclcpp::spin(node);
    //shutdown ros
    rclcpp::shutdown();
    return 0;
}






//statistical inferencing {

0, 0, 0, 0, 8000, 8000, 8000, 8000, 0, 0, 0, 0, 8000, 8000, 8000, 8000, 0

// subtract second value from first and take absolute
0, 0, 0, 0, 8000, 0, 0, 0, 8000, 0, 0, 0, 8000, 0, 0, 0, 8000, 0

//replace all 8000's with 1's 
0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0

// sum the array 
4

//divide sum by 2
2

//}