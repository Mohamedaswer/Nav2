#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "basic_mobile_robot/action/order_delivery.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class OrderDeliveryServer : public rclcpp::Node {
public:
    using OrderDelivery = basic_mobile_robot::action::OrderDelivery;
    using GoalHandleOrderDelivery = rclcpp_action::ServerGoalHandle<OrderDelivery>;

    OrderDeliveryServer() : Node("order_delivery_server") {
        this->action_server_ = rclcpp_action::create_server<OrderDelivery>(
            this,
            "order_delivery",
            std::bind(&OrderDeliveryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OrderDeliveryServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&OrderDeliveryServer::handle_accepted, this, std::placeholders::_1)
        );

        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&OrderDeliveryServer::amcl_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Order Delivery Action Server with Navigation Started");
    }

private:
    rclcpp_action::Server<OrderDelivery>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;

    geometry_msgs::msg::Pose current_pose_;

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_pose_ = msg->pose.pose;  // Update current robot position
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const OrderDelivery::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received order for table: %s", goal->table_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOrderDelivery> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Cancel request received. Stopping robot...");
        publish_goal("home");  // Send robot back home
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleOrderDelivery> goal_handle) {
        std::thread(std::bind(&OrderDeliveryServer::execute, this, goal_handle)).detach();
    }

    void execute(const std::shared_ptr<GoalHandleOrderDelivery> goal_handle) {
        auto feedback = std::make_shared<OrderDelivery::Feedback>();
        auto result = std::make_shared<OrderDelivery::Result>();

        std::string table = goal_handle->get_goal()->table_id;
        bool require_confirmation = goal_handle->get_goal()->require_confirmation;

        RCLCPP_INFO(this->get_logger(), "Moving to kitchen...");
        publish_goal("kitchen");
        wait_for_reach("kitchen");

        if (require_confirmation) {
            RCLCPP_INFO(this->get_logger(), "Waiting for confirmation at the kitchen...");
            if (!wait_for_confirmation(10)) {  // Timeout in 10 seconds
                result->success = false;
                result->result_status = "No confirmation at kitchen, returning home";
                goal_handle->succeed(result);
                publish_goal("home");
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Moving to table: %s", table.c_str());
        publish_goal(table);
        wait_for_reach(table);

        if (require_confirmation) {
            RCLCPP_INFO(this->get_logger(), "Waiting for confirmation at table...");
            if (!wait_for_confirmation(10)) {
                result->success = false;
                result->result_status = "No confirmation at table, returning to kitchen";
                goal_handle->succeed(result);
                publish_goal("kitchen");
                wait_for_reach("kitchen");
                publish_goal("home");
                return;
            }
        }

        result->success = true;
        result->result_status = "Order delivered successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Order completed. Returning home...");
        publish_goal("home");
    }

    void publish_goal(const std::string &location) {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";

        if (location == "home") {
            goal_msg.pose.position.x = 0.0;
            goal_msg.pose.position.y = 0.0;
        } else if (location == "kitchen") {
            goal_msg.pose.position.x = 2.0;
            goal_msg.pose.position.y = 0.0;
        } else if (location == "table1") {
            goal_msg.pose.position.x = 4.0;
            goal_msg.pose.position.y = 2.0;
        } else if (location == "table2") {
            goal_msg.pose.position.x = 4.0;
            goal_msg.pose.position.y = 4.0;
        } else if (location == "table3") {
            goal_msg.pose.position.x = 4.0;
            goal_msg.pose.position.y = 6.0;
        }

        goal_publisher_->publish(goal_msg);
    }

    void wait_for_reach(const std::string &location) {
        RCLCPP_INFO(this->get_logger(), "Waiting to reach %s...", location.c_str());
        double target_x = 0.0, target_y = 0.0;
        if (location == "kitchen") { target_x = 2.0; target_y = 0.0; }
        else if (location == "table1") { target_x = 4.0; target_y = 2.0; }
        else if (location == "table2") { target_x = 4.0; target_y = 4.0; }
        else if (location == "table3") { target_x = 4.0; target_y = 6.0; }

        while (rclcpp::ok()) {
            double dx = current_pose_.position.x - target_x;
            double dy = current_pose_.position.y - target_y;
            if (std::sqrt(dx * dx + dy * dy) < 0.3) {
                RCLCPP_INFO(this->get_logger(), "Reached %s", location.c_str());
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    bool wait_for_confirmation(int timeout_seconds) {
        RCLCPP_INFO(this->get_logger(), "Waiting for confirmation (Press ENTER in terminal)...");
        std::string input;
        auto start_time = this->now();
        while ((this->now() - start_time).seconds() < timeout_seconds) {
            if (std::getline(std::cin, input)) {
                RCLCPP_INFO(this->get_logger(), "Confirmation received!");
                return true;
            }
        }
        RCLCPP_WARN(this->get_logger(), "Timeout: No confirmation received.");
        return false;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderDeliveryServer>());
    rclcpp::shutdown();
    return 0;
}

