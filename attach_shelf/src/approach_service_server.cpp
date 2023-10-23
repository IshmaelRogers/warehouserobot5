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
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

struct Point {
    double x;
    double y;
};

struct Leg {
    int startIndex;
    int endIndex;
    int middleIndex;
    Point position;
};

//create serviceServer class
class serviceServer : public rclcpp::Node
{
    public:
    serviceServer() : Node("service_server")
    {
        
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&serviceServer::laserCallback, this, _1));
        srv_ = create_service<std_srvs::srv::SetBool>("attach_shelf", std::bind(&serviceServer::handleService, this, std::placeholders::_1, std::placeholders::_2));
        

    }

    private:

    void handleService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {

        RCLCPP_INFO(this->get_logger(), "Requested /attach_shelf Service: %d", request->data);
        if (request->data == true){

        detectLeg();    
        
        }else{
        
            RCLCPP_INFO(this->get_logger(), "Server Not Started: ");
        
        }
      
      
      }

    void detectLeg(){

    //loop through the intensities
    bool insideLeg = false;
    int currentStartIndex = -1;

    for (std::size_t i = 0; i < intensities_.size(); ++i) {
        if (intensities_[i] == 8000 && (i == 0 || intensities_[i - 1] != 8000)) {
            insideLeg = true;
            currentStartIndex = i;
            //display the start of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected start of leg at index %zu", i);

        } else if (insideLeg && intensities_[i] != 8000) {
            int middle = (currentStartIndex + i - 1) / 2;
            //display the middle of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected middle of leg at index %d", middle);
            legs.push_back({currentStartIndex, static_cast<int>(i - 1), middle, {0.0, 0.0}});
            insideLeg = false;
            //display the end of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected end of leg at index %zu", (i - 1));

        }
    }

    if (insideLeg) {
        int middle = (currentStartIndex + intensities_.size() - 1) / 2;
        legs.push_back({currentStartIndex, static_cast<int>(intensities_.size()) - 1, middle, {0.0, 0.0}});
    }
    
    
    }
   
   void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // store the laser message
    laser_msg_ = scan;
    // store the laser intensities
    intensities_ = scan->intensities;
    // store the laser ranges
    ranges_ = scan->ranges;
    
}

    //create variable for the laser intensities
    std::vector<float> intensities_;
    //create variable for the laser ranges
    std::vector<float> ranges_;
    //create a subscriber for the laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;
    //vector for legs
    std::vector<Leg> legs;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
    


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

//0, 0, 0, 0, 8000, 8000, 8000, 8000, 0, 0, 0, 0, 8000, 8000, 8000, 8000, 0

// subtract second value from first and take absolute
//0, 0, 0, 0, 8000, 0, 0, 0, 8000, 0, 0, 0, 8000, 0, 0, 0, 8000, 0

//replace all 8000's with 1's 
//0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0

// sum the array 
//4

//divide sum by 2
//2

//}