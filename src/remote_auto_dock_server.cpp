/*
 * service 任意定位点返回充电桩对接充电, 流程节点整合
 */
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/srv/dock_server.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace auto_dock_server_extension
{
class RemoteAutoDockServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    RemoteAutoDockServer(): 
    Node("remote_auto_dock_server")
    // send_goal_thread_1(nullptr)
    {
        dock_battery_level_th_ =  this->declare_parameter<float>("dock_battery_level_th", 0.01);
        dock_pose_ = this->declare_parameter<std::vector<float>>("dock_pose", std::vector<float>{0.0, 0.0, 0.0});

        service_ds_ = this->create_service<robot_interfaces::srv::DockServer>(
            "/service_dock",
            std::bind(&RemoteAutoDockServer::handle_request_ds, this, std::placeholders::_1, std::placeholders::_2));

        remote_auto_dock_trigger_tag = false;
        is_near_dock_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/is_near_dock", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        remote_auto_dock_trigger_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/android_voice_dock", 10, std::bind(&RemoteAutoDockServer::handle_remote_auto_dock_trigger, this, std::placeholders::_1));

        nav2_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "remote_auto_dock_server is running..");

    }

    ~RemoteAutoDockServer(){
        rclcpp::shutdown();
    }

    void handle_request_ds(
        const std::shared_ptr<robot_interfaces::srv::DockServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::DockServer::Response> response)
    {
        switch (request->cmd_name) {
            case 1:  // start auto dock 
                try{
                    start_auto_dock(response);  
                }catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error in start_auto_dock: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "start_auto_dock error: %s", e.what());
                    break;
                } catch (...) {
                    response->result = false;
                    response->message = "Unknown error in start_auto_dock ";
                    RCLCPP_ERROR(get_logger(), "Unknown error in start_auto_dock");
                    break;
                }
                response->result = true;
                response->message = std::string("start_auto_dock successfully");
                break;
            case 2:  // Stop auto dock
                try{
                    stop_auto_dock(response);  
                }catch (const std::exception& e) {
                    response->result = false;
                    response->message = std::string("Error in stop_auto_dock: ") + e.what();
                    RCLCPP_ERROR(get_logger(), "stop_auto_dock error: %s", e.what());
                    break;
                } catch (...) {
                    response->result = false;
                    response->message = "Unknown error in stop_auto_dock ";
                    RCLCPP_ERROR(get_logger(), "Unknown error in stop_auto_dock");
                    break;
                }
                response->result = true;
                response->message = std::string("stop_auto_dock successfully");
                break;
            default:
                response->result = false;
                response->message = "Invalid command";
                break;
        
    }

    void start_auto_dock(std::shared_ptr<robot_interfaces::srv::DockServer::Response> response){

        if (!nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
            throw std::runtime_error("Navigation2 Action server not available after waiting");
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = dock_pose_[0];
        goal_msg.pose.pose.position.y = dock_pose_[1];
        double pose_yaw = dock_pose_[2];
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, pose_yaw);  // Create quaternion from yaw (around Z-axis)
        quat.normalize();  // Ensure it's normalized
        goal_msg.pose.pose.orientation.z = quat.z();
        goal_msg.pose.pose.orientation.w = quat.w();
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "navigation2 Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&RemoteAutoDockServer::nav2_goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&RemoteAutoDockServer::nav2_feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&RemoteAutoDockServer::nav2_result_callback, this, _1);
        nav2_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    }

    void stop_auto_dock(std::shared_ptr<robot_interfaces::srv::DockServer::Response> response){

        if (!goal_handle_) {
            throw std::runtime_error("No active navigation to pause");
        }
        
        auto future_cancel = nav2_client_ptr_->async_cancel_goal(goal_handle_);
    }

    // void nav2_send_goal()
    // {
    //     while(rclcpp::ok()){
    //         while(!remote_auto_dock_trigger_tag){
    //             sleep(1);
    //             RCLCPP_INFO(this->get_logger(), "no remote auto dock trigger");
    //         }
    //         using namespace std::placeholders;
    //         if (!nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    //             RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
    //             // rclcpp::shutdown();
    //             return;
    //         }
    //         auto goal_msg = NavigateToPose::Goal();
    //         goal_msg.pose.pose.position.x = 0.59847f;
    //         goal_msg.pose.pose.position.y = 1.10564f;
    //         goal_msg.pose.pose.orientation.z = 0.662676f;
    //         goal_msg.pose.pose.orientation.w = 0.748906f;
    //         goal_msg.pose.header.frame_id = "map";
    //         RCLCPP_INFO(this->get_logger(), "navigation2 Sending goal");
    //         auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    //         send_goal_options.goal_response_callback =
    //             std::bind(&RemoteAutoDockServer::nav2_goal_response_callback, this, _1);
    //         send_goal_options.feedback_callback =
    //             std::bind(&RemoteAutoDockServer::nav2_feedback_callback, this, _1, _2);
    //         send_goal_options.result_callback =
    //             std::bind(&RemoteAutoDockServer::nav2_result_callback, this, _1);
    //         nav2_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    //         // RCLCPP_INFO(this->get_logger(), "navigation2 Sent goal");
    //         remote_auto_dock_trigger_tag = false;
    //     }    
    // }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;
    // bool IsNearDock = false;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_near_dock_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std_msgs::msg::UInt8 is_near_dock_msg;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr remote_auto_dock_trigger_sub_;
    bool remote_auto_dock_trigger_tag;
    rclcpp::Service<robot_interfaces::srv::DockServer>::SharedPtr service_ds_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    float dock_battery_level_th_;
    std::vector<float> dock_pose_;


    //navigation2
    void nav2_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was rejected by server");
            throw std::runtime_error("Navigation2 Goal was rejected by server");
        } else {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Navigation2 Goal accepted by server, waiting for result");
        }
    }

    void nav2_feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);

        is_near_dock_msg.data = 0; 
        is_near_dock_pub_->publish(is_near_dock_msg);
    }
    
    void nav2_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation2 Goal reached");

            // dock_send_goal();
            // IsNearDock = true;
            //发布到topic /is_near_dock 1  与 huamai base_driver联动
            is_near_dock_msg.data = 1;  // 设置 UInt8 数据
            for (int ii=0; ii<3; ii++){
                is_near_dock_pub_->publish(is_near_dock_msg);
            }
            RCLCPP_INFO(this->get_logger(), "Navigation2 nav2_result_callback is done");
            break;
            // return;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was aborted");
            break;
            // return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was canceled");
            break;
            // return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
            // return;
        }

        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        for(int ii=0; ii < 10; ii++){
            cmd_vel_pub_->publish(msg);
            sleep(0.01);
        }

    }

    void handle_remote_auto_dock_trigger(const std_msgs::msg::UInt8::ConstSharedPtr msg){
        if(msg->data == 2){
            remote_auto_dock_trigger_tag = true;
        }else{
            remote_auto_dock_trigger_tag = false;
        }     
    }

};  // class RemoteAutoDockServer
} //namespace auto_dock_server_extension

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_dock_server_extension::RemoteAutoDockServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}