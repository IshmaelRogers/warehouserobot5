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

    }

    private:

    //memeber variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Service<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr approach_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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

   void approachCallback(const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Response> response)
    {
        
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
