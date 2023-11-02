#include "checkpoint5_interfaces/srv/detail/go_to_loading__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <mutex>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ServiceClientNode : public rclcpp::Node
{
    public:
        ServiceClientNode() : Node("ServiceClientNode")
        {
            // parameter declaration
            //obstacle parameter
            auto obstacleParam_desc = rcl_interfaces::msg::ParameterDescriptor{};
            obstacleParam_desc.description = "obstacle parameter";
            this->declare_parameter<double>("obstacle", 0.3, obstacleParam_desc);

            //degree parameter
            auto degreeParam_desc = rcl_interfaces::msg::ParameterDescriptor{};
            degreeParam_desc.description = "degree parameter";
            this->declare_parameter<double>("degree", 90.0, degreeParam_desc);

            //approach parameter
            auto approachParam_desc = rcl_interfaces::msg::ParameterDescriptor{};
            approachParam_desc.description = "approach parameter";
            this->declare_parameter<bool>("approach", 0.3, approachParam_desc);

            // callback group clbg
            clbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);  
            //subscription Options
            rclcpp::SubscriptionOptions subOptions;
            subOptions.callback_group = clbg;

            //subscribers
            laserSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&ServiceClientNode::laserCallback, this, _1), subOptions); 

            //odometry subscriber
            odometrySubscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&ServiceClientNode::odometryCallback, this, _1), subOptions);

            //publishers
            //velocity publisher to the robot/cmd_vel topic
            velocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

            //timer 
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ServiceClientNode::timerCallback, this));

            //client to call the service
            client_ = this->create_client<checkpoint5_interfaces::srv::GoToLoading>("approach_shelf");
        }
    private:
    //parameter variables
    double obstacle;
    double degree;
    bool approach;

    int front;
    bool final = false;
    float front_range = 0.0;
    double rotation = 0.0;
    double desired_rotation = 0.0;
    //switch statement variable
    int phase = 1;

    //velocity message
    geometry_msgs::msg::Twist cmd;

    //declare subscriber 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber;

    //declare publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;

    //declare timer
    rclcpp::TimerBase::SharedPtr timer_;

    //declare client
    rclcpp::Client<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr client_;

    //declare callback group
    rclcpp::CallbackGroup::SharedPtr clbg;


    //*******helper functions
    double deg2rad(double degree){
        return degree * M_PI / 180;
    }

    bool positioned(){
    rotation = round(rotation*1000)/10000;
    desired_rotation = round(desired_rotation*1000)/10000;
    //if desired rotation is greater than 0 and rotation is greater than or equal to desired rotation
    if(desired_rotation > 0 && rotation >= desired_rotation){
        return true;
    } else if(desired_rotation < 0 && rotation <= desired_rotation){
        //if desired rotation is less than 0 and rotation is less than or equal to desired rotation
        return true;
    } else {
        //if neither of the above, return false
        return false;
        }
    }
    //********


    //laser callback definition
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {   
        front = round(scan->angle_max*2/scan->angle_increment)/2;
        front_range = scan->ranges[front];
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry){
        //get the current orientation
        auto orientation = odometry->pose.pose.orientation;
        //convert to quaternion
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        //convert to euler
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        rotation = yaw;
    }
    void timerCallback()
    {
        //get the parameter values
        
        this->get_parameter("obstacle", obstacle);
        
        this->get_parameter("degree", degree);

        this->get_parameter("approach", approach);
        //call deg2rad function using desired_rotation to convert degree to radian
        desired_rotation = deg2rad(degree);

        //declare cmd velocity constants
        const static float xMax = 0.5;
        const static float zMax = 0.5;
        const static float zMin = 0.1;

        //create a switch statement to switch between phases
        //initialize the service request
        auto request = std::make_shared<checkpoint5_interfaces::srv::GoToLoading::Request>();
        switch(phase){
            case 1:

                //robot is moving forward and not turning 
                cmd.angular.z = 0.0;
                cmd.linear.x = (front_range - obstacle) * 0.75;
                //if linear velocity is greater than xMax, set it to xMax
                if(cmd.linear.x > xMax){
                    cmd.linear.x = xMax;
                }
                //if front_range is less than obstacle, stop the robot and change phase
                if(front_range < obstacle){
                    cmd.linear.x = 0.0;
                    //display "Stopped. Ready to rotate into position" to the terminal
                    RCLCPP_INFO(this->get_logger(), "Stopped. Ready to rotate into position");
                    //change phase
                    phase = 2;
                }
                break;
            case 2:
                //robot is ready to turn to the desired_rotation
                cmd.linear.x = 0.0;
                cmd.angular.z = (desired_rotation - rotation) * 0.25;
                //if the difference between desired_rotation and rotation is greater than 0.0, stop the robot 
                if((desired_rotation - rotation) > 0.0){
                    //if angular velocity is greater than zMax, set it to zMax
                    if(cmd.angular.z > zMax){
                        cmd.angular.z = zMax;
                    } else if(cmd.angular.z < zMin){
                        //if angular velocity is less than zMin, set it to zMin
                        cmd.angular.z = zMin;
                    }
                } else if (desired_rotation - rotation < 0.0){
                    //if the difference between desired_rotation and rotation is less than 0.0, stop the robot 
                    //if angular velocity is less than -zMax, set it to -zMax
                    if(cmd.angular.z < -zMax){
                        cmd.angular.z = -zMax;
                    } else if(cmd.angular.z > -zMin){
                        //if angular velocity is greater than -zMin, set it to -zMin
                        cmd.angular.z = -zMin;
                    }
                }
                //if the robot is positioned, stop the robot and change phase
                if(positioned()){
                    cmd.angular.z = 0.0;
                    //display "Positioned. Ready to approach shelf" to the terminal
                    RCLCPP_INFO(this->get_logger(), "Positioned. Ready to approach shelf");
                    //change phase
                    phase = 3;
                }
                break;
            case 3:

                //display "Approaching shelf" to the terminal
                RCLCPP_INFO(this->get_logger(), "Approaching shelf");
                
                //set the request to the approach parameter
                request->attach_to_shelf = approach;
                //while final is false, call the service
                while(!final){
                    //while client is not ready, wait
                    while(!client_->wait_for_service(1s)){
                            RCLCPP_INFO(this->get_logger(), "service not avaialable, waiting again...");
                    }
                    //call the service
                    auto res_future = client_->async_send_request(request);
                    //wait for the result
                    std::future_status status = res_future.wait_for(5s);
                    // if status is ready display respond recieved
                    if(status == std::future_status::ready){
                        RCLCPP_INFO(this->get_logger(), "Response recieved");
                        //assign response to res_future.get()
                        auto result = res_future.get();
                        //if result is set to complete
                        if(result->complete){
                            //display "Approach complete" to the terminal
                            RCLCPP_INFO(this->get_logger(), "Approach complete");
                            //set final to true
                            final = true;
                        }else if(!result->complete){
                        //if result is not set to complete, display "Approach failed" to the terminal
                        RCLCPP_INFO(this->get_logger(), "Approach failed");
                        //set final to true
                        final = true;
                    }
                  }
                }
                //change phase
                phase = 4;
                break;
            case 4:
                //display "Approach complete" to the terminal
                RCLCPP_INFO(this->get_logger(), "Approach complete");
            break;
    
            default:
                //display IDLE to the terminal
                RCLCPP_INFO(this->get_logger(), "IDLE");
                if(!final){
                    velocityPublisher->publish(cmd);
                }
            
        }
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClientNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}