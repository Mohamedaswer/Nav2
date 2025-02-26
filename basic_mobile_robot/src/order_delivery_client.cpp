#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "basic_mobile_robot/action/order_delivery.hpp"

class OrderDeliveryClient : public rclcpp::Node {
public:
    using OrderDelivery = basic_mobile_robot::action::OrderDelivery;
    using GoalHandleOrderDelivery = rclcpp_action::ClientGoalHandle<OrderDelivery>;

    OrderDeliveryClient() : Node("order_delivery_client") {
        this->client_ = rclcpp_action::create_client<OrderDelivery>(this, "order_delivery");
    }

    void send_goal(const std::string &table, bool require_confirmation) {
        if (!this->client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = OrderDelivery::Goal();
        goal_msg.table_id = table;
        goal_msg.require_confirmation = require_confirmation;

        RCLCPP_INFO(this->get_logger(), "Sending goal to deliver order to %s", table.c_str());

        auto send_goal_options = rclcpp_action::Client<OrderDelivery>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&OrderDeliveryClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&OrderDeliveryClient::result_callback, this, std::placeholders::_1);

        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<OrderDelivery>::SharedPtr client_;

    void feedback_callback(
        GoalHandleOrderDelivery::SharedPtr, const std::shared_ptr<const OrderDelivery::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
    }

    void result_callback(const GoalHandleOrderDelivery::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Order delivered successfully!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Order failed: %s", result.result->result_status.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("client"), "Usage: order_delivery_client <table_id> [confirmation]");
        return 1;
    }

    std::string table = argv[1];
    bool require_confirmation = (argc >= 3 && std::string(argv[2]) == "confirm");

    auto client = std::make_shared<OrderDeliveryClient>();
    client->send_goal(table, require_confirmation);

    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}

