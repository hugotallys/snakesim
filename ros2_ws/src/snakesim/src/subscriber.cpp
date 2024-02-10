#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "snakesim_interfaces/msg/joint_state.hpp"
#include "snakesim_interfaces/srv/spawn_snake.hpp"

#include "SnakeSim.hpp"

#define WINDOW_WIDTH 700
#define WINDOW_HEIGHT 700

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber() : Node("my_subscriber"), sim(std::make_shared<SnakeSim>()), simThread([this]() { this->sim->run(); })
    {
        subscription_ = this->create_subscription<snakesim_interfaces::msg::JointState>(
            "joint_state",
            10,
            std::bind(&MySubscriber::topicCallback, this, std::placeholders::_1));

        service_ = this->create_service<snakesim_interfaces::srv::SpawnSnake>("spawn_snake",
            std::bind(&MySubscriber::spawn_snake, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~MySubscriber()
    {
        sim->stop();
        
        if (simThread.joinable()) {
            simThread.join();
        }

        subscription_.reset();
    }

private:
    void topicCallback(const snakesim_interfaces::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received position = [%f, %f, %f]", msg->position[0], msg->position[1], msg->position[2]);
        RCLCPP_INFO(this->get_logger(), "Received eigen_values = [%f, %f]", msg->eigen_values[0], msg->eigen_values[1]);
        sim->setAngles(msg->position);
        sim->setEigenValues(msg->eigen_values);
        sim->setEigenAngle(msg->eigen_angle);
    }

    void spawn_snake(
        const std::shared_ptr<snakesim_interfaces::srv::SpawnSnake::Request> request,
        std::shared_ptr<snakesim_interfaces::srv::SpawnSnake::Response>      response
    )
    {
        if (!sim->getSnakeCount()) {
            sim->createSnake(
                request->origin,
                request->position
            );
            response->success = true;
        } else {
            response->success = false;
        }
    }

    rclcpp::Subscription<snakesim_interfaces::msg::JointState>::SharedPtr subscription_;
    rclcpp::Service<snakesim_interfaces::srv::SpawnSnake>::SharedPtr service_;

    std::shared_ptr<SnakeSim> sim;
    std::thread simThread;
};

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MySubscriber>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}