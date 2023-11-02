#include "geometry_msgs/msg/twist.hpp"
#include "checkpoint5_interfaces/srv/detail/go_to_loading__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <functional>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/msg/empty.hpp"
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::vector;

// define a point structure with double x and y coordinates
struct Point
{
  double x;
  double y;
}; 

// define a leg structure with a startIndex, endIndex, middleIndex and a point position
struct Leg
{
  int startIndex;
  int endIndex;
  int middleIndex;
  Point position;
};

//create a class called serviceServer that inherits from rclcpp::Node
class serviceServer : public rclcpp::Node
{
    public:
    serviceServer() : Node("service_server")
    {
        //create a reentrant callback group
        clbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions subOptions;
        subOptions.callback_group = clbg;
        //create a subscription to the scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&serviceServer::scanCallback, this, _1), subOptions);
        //create a subscription to the odom topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&serviceServer::odomCallback, this, _1), subOptions);
        //create a service to go to the loading zone
        approach_service_ = create_service<checkpoint5_interfaces::srv::GoToLoading>("approach_shelf", std::bind(&serviceServer::approachCallback, this, _1, _2));
        // create a velocity publisher
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        //create a tf buffer
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        //create tf listener
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        
        //create a publisher for elevator up and down
        elevator_up_pub_ = this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
        elevator_down_pub_ = this->create_publisher<std_msgs::msg::Empty>("/elevator_down", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        //tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)

    }

    private:

    //memeber variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Service<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr approach_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_down_pub_;

    //broadcaster for cartTF
    //static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::CallbackGroup::SharedPtr clbg;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};


    int front;
    int num_rays;
    double front_range=0.0;
    vector <float> front_intensities_;
    double current_x;
    double current_y;
    vector<Leg> legs;
    vector<Leg> detectedLegs;
    Point midPoint;
    Leg detectedLeg1;
    Leg detectedLeg2;
    vector<float> laser_range;

    //leg detection functions start here//

    // a function that returns point and calculates the position of the leg
    Point calculatePosition(double distance, int ray_index, int num_rays)
    {
        double angle_deg = (ray_index * 360.0) / num_rays;
        double angle_rad = angle_deg * (M_PI / 180.0); // Convert to radians

        Point position;
        position.x = distance * cos(angle_rad);
        position.y = distance * sin(angle_rad);

        return position;
    }

    void assignLegPoints(std::vector<Leg>& detectedLegs, const std::vector<float>& rangeValues, int num_rays) {
    for (Leg& leg : detectedLegs) {
        leg.position = calculatePosition(rangeValues[leg.middleIndex], leg.middleIndex, num_rays);
    }
}


    vector<Leg> detectLegs(vector<float>& intensities)
    {
        bool insideLeg = false;
        int currentStartIndex = -1;

        for (int i : intensities) {
            if (intensities[i] == 8000 && (i == 0 || intensities[i - 1] != 8000)) {
                insideLeg = true;
                currentStartIndex = i;
            } else if (insideLeg && intensities[i] != 8000) {
                int middle = (currentStartIndex + i - 1) / 2;
                Leg temp_leg;
                temp_leg.startIndex = currentStartIndex;
                temp_leg.endIndex = i - 1;
                temp_leg.middleIndex = middle;
                legs.push_back(temp_leg);
                insideLeg = false;
            }
        }

        if (insideLeg) {
            int middle = (currentStartIndex + intensities.size() - 1) / 2;
            Leg temp_leg;
            temp_leg.startIndex = currentStartIndex;
            temp_leg.endIndex = static_cast<int>(intensities.size()) - 1;
            temp_leg.middleIndex = middle;
            legs.push_back(temp_leg);
        }
        return legs;
    }

    double distanceBetweenLegs(const Leg& leg1, const Leg& leg2) 
    {
        return euclideanDistance(leg1.position, leg2.position);
    }

    Point midpoint(const Point& p1, const Point& p2)
    {
        Point middle;
        middle.x = (p1.x + p2.x) / 2.0;
        middle.y = (p1.y + p2.y) / 2.0;
        return middle;
    }

    Point midpointBetweenLegs(const Leg& detectedLeg1, const Leg& detectedLeg2)
    {
        return midpoint(detectedLeg1.position, detectedLeg2.position);
    }

    void processLaserData()
    {
        detectedLegs = detectLegs(front_intensities_);
        assignLegPoints(detectedLegs, laser_range, front);
        if (detectedLegs.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Less than two legs detected!");
            return;  // Exit the function if there aren't at least two legs
        }
        detectedLeg1 = detectedLegs[0];
        detectedLeg2 = detectedLegs[1];
        midPoint = midpointBetweenLegs(detectedLegs[0], detectedLegs[1]);
        // Printing the results
    for (const Leg& leg : detectedLegs) {
        // Print the leg information using RCLCPP_INFO
        RCLCPP_INFO(this->get_logger(), "Leg: Start index = %d, Middle index = %d, End index = %d, Position = (%f, %f)", leg.startIndex, leg.middleIndex, leg.endIndex, leg.position.x, leg.position.y);

    }

    // Print the midpoint and distance between legs using RCLCPP_INFO
    RCLCPP_INFO(this->get_logger(), "Midpoint between legs: (%f, %f)", midPoint.x, midPoint.y);

    double dist = distanceBetweenLegs(detectedLeg1, detectedLeg2);
    RCLCPP_INFO(this->get_logger(), "Distance between detected legs: %f", dist);
    }


    double euclideanDistance(const Point& p1, const Point& p2) 
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    //leg detection functions end here//


    // create scanCallback function that takes in a sensor_msgs::msg::LaserScan::SharedPtr msg
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        front = round(scan->angle_max / scan->angle_increment)/2;
        front_range = scan->ranges[front];
        front_intensities_ = scan->intensities;
        num_rays = scan->ranges.size();
        laser_range = scan->ranges;
    }

    // create odomCallback function that takes in a nav_msgs::msg::Odometry::SharedPtr msg
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //get current x and y position of the robot
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

    }

    //function to set the cartTF frame
    void cartTFSet(){

        geometry_msgs::msg::TransformStamped cartTF;    
        
        cartTF.header.stamp = this->now();
        cartTF.header.frame_id = "robot/base_link";
        cartTF.child_frame_id = "cartTF";
        cartTF.transform.translation.x = midPoint.x;
        cartTF.transform.translation.y = midPoint.y;
        cartTF.transform.translation.z = 0.0;
        cartTF.transform.rotation.x = 0.0;
        cartTF.transform.rotation.y = 0.0;
        cartTF.transform.rotation.z = 0.0;
        cartTF.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(cartTF);
        

    }

    void moveRobotToCartTF()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        while (rclcpp::ok()) 
        {
            try 
            {
                transformStamped = tfBuffer->lookupTransform("robot/base_link", "cart_frame", rclcpp::Time(0));
            } 
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                rclcpp::sleep_for(500ms);
                continue;
            }

            // Calculate the difference between the cart frame position and the current robot position
            double dx = transformStamped.transform.translation.x - current_x;
            double dy = transformStamped.transform.translation.y - current_y;

            // Calculate yaw angle to target
            double target_yaw = atan2(dy, dx);
            
            // Create a twist message
            geometry_msgs::msg::Twist cmd;
            double xMax = 0.5, xMin = 0.05, zMax = 0.5, zMin = 0.05;
            double linear_threshold = 0.1, angular_threshold = 0.05; // Example threshold values.

            // Linear control for moving towards the cart frame.
            if (fabs(dx) > linear_threshold || fabs(dy) > linear_threshold) 
            {
                cmd.linear.x = std::sqrt(dx*dx + dy*dy) * 0.25; // Distance to target * some control gain
                
                if (cmd.linear.x > xMax) 
                {
                    cmd.linear.x = xMax;
                } 
                else if (cmd.linear.x < xMin) 
                {
                    cmd.linear.x = xMin;
                }
            } 
            else 
            {
                cmd.linear.x = 0.0;
            }

            // Angular control to face towards the cart frame.
            if (fabs(target_yaw) > angular_threshold) 
            {
                cmd.angular.z = target_yaw * 0.3; // Yaw difference * some control gain
                
                if (cmd.angular.z > zMax) 
                {
                    cmd.angular.z = zMax;
                } 
                else if (cmd.angular.z < zMin) 
                {
                    cmd.angular.z = zMin;
                }
            } 
            else 
            {
                cmd.angular.z = 0.0;
            }

            vel_pub_->publish(cmd);
            
            // Break condition
            if (fabs(dx) < 0.1 && fabs(dy) < 0.1)
            {
                break;
            }

            // Sleep for 100 milliseconds
            rclcpp::sleep_for(100ms);
        }
    }

    //function to move robot forward
    void moveRobotForward()
    {
        // Desired distance to move forward
        double desired_distance = 0.30; // 30 cm
        double starting_x = current_x;
        double starting_y = current_y;

        while (rclcpp::ok()) 
        {
            // Calculate the distance traveled since starting the move
            double dx = current_x - starting_x;
            double dy = current_y - starting_y;
            double traveled_distance = std::sqrt(dx*dx + dy*dy);

            // Create a twist message
            geometry_msgs::msg::Twist cmd;
            double xMax = 0.5, xMin = 0.05;
            double linear_threshold = 0.02; // Slightly smaller threshold to ensure accurate stopping

            // Check if the desired distance is reached or almost reached
            if (traveled_distance < (desired_distance - linear_threshold)) 
            {
                double remaining_distance = desired_distance - traveled_distance;
                cmd.linear.x = remaining_distance * 0.25; // Distance remaining * control gain
                
                if (cmd.linear.x > xMax) 
                {
                    cmd.linear.x = xMax;
                } 
                else if (cmd.linear.x < xMin) 
                {
                    cmd.linear.x = xMin;
                }
            } 
            else 
            {
                cmd.linear.x = 0.0;
            }

            vel_pub_->publish(cmd);

            // Break condition: If we have traveled the desired distance or more, stop moving
            if (traveled_distance >= desired_distance)
            {
                break;
            }

            // Sleep for 100 milliseconds
            rclcpp::sleep_for(100ms);
        }
    }

    //function to move elevator up
    void raiseElevator()
    {
        std_msgs::msg::Empty msg;
        elevator_up_pub_->publish(msg);
    }

    void lowerElevator()
    {
        std_msgs::msg::Empty msg;        
        elevator_down_pub_->publish(msg);
    }





    void approachCallback(const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Response> response)
    {
        if (request->attach_to_shelf)
        {
            processLaserData();

            if (detectedLegs.size() < 2)
            {
                response->complete = false;
                return;
            }

            //set the cart TF
            cartTFSet();

            // output if cart set was successful 

            //move robot to set point 
            moveRobotToCartTF();

            //move robot another 30 cm forward
            moveRobotForward();

            //elevator up function
            raiseElevator();

            //elevator down function
            lowerElevator();

        }
        else
        {
            cartTFSet();
            response->complete = false;
        }
    }

};

int main(int argc, char **argv)
{
    //initialize ros
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<serviceServer>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}