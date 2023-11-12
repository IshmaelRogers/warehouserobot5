#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <utility>
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/callback_group.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::vector;


class ApproachServiceServer : public rclcpp::Node {
public:
    ApproachServiceServer() : Node("approach_service") {
        //create a reentrant callback group
        clbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions subOptions;
        subOptions.callback_group = clbg;
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&ApproachServiceServer::scanCallback, this, std::placeholders::_1));
        //create a service to go to the loading zone
        approach_service_ = create_service<checkpoint5_interfaces::srv::GoToLoading>("approach_shelf", std::bind(&ApproachServiceServer::approachCallback, this, _1, _2), rmw_qos_profile_services_default, clbg);
        
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        laser_ranges_ = msg->ranges;
        laser_intensities_ = msg->intensities;
        
    }

    std::pair<std::vector<int>, std::vector<int>> findBunchIndices(const std::vector<float>& intensities, float value = 8000.0) {
        std::vector<int> bunchStarts;
        std::vector<int> bunchEnds;
        bool inBunch = false;

        for (int i = 0; i < intensities.size(); ++i) {
            if (intensities[i] == value && !inBunch) {
                bunchStarts.push_back(i);
                inBunch = true;
            } else if (intensities[i] != value && inBunch) {
                bunchEnds.push_back(i - 1);
                inBunch = false;
            }
        }

        if (inBunch) {
            bunchEnds.push_back(intensities.size() - 1);
        }

        return {bunchStarts, bunchEnds};
    }

    void approachCallback(const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Response> response)
    {
        if (request->attach_to_shelf)
        {
            auto [starts, ends] = findBunchIndices(laser_intensities_);
            
            // Process the results as needed
            for (size_t i = 0; i < starts.size(); ++i) {
            int startIdx = starts[i];
            int endIdx = ends[i];
            float startIntensity = laser_intensities_[startIdx];
            float endIntensity = laser_intensities_[endIdx];
                RCLCPP_INFO(this->get_logger(), "Bunch %lu: Start %d (Intensity: %f), End %d (Intensity: %f)", 
                        i, startIdx, startIntensity, endIdx, endIntensity);
            }

            if (starts.size() < 2)
            {
                response->complete = false;
                RCLCPP_INFO(this->get_logger(), "Less than two legs found");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "2 Legs found: Publish the cart frame");

            //set the cart TF
            //cartTFSet();

            // output if cart set was successful 

            //move robot to set point 
           // moveRobotToCartTF();

            //move robot another 30 cm forward
           // moveRobotForward();

            //elevator up function
            //raiseElevator();

            //elevator down function
           // lowerElevator();

        }
        else
        {
            //cartTFSet();
            response->complete = false;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::vector<float> laser_ranges_;
    std::vector<float> laser_intensities_;
    rclcpp::Service<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr approach_service_;
    rclcpp::CallbackGroup::SharedPtr clbg;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachServiceServer>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
