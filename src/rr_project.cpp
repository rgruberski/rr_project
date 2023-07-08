#include <string>
#include <cstdlib>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#define DISTANCE_TOLERANCE 0.1
#define ANGLE_TOLERANCE 0.01

using std::placeholders::_1;
using namespace std::chrono_literals;

enum TurtleState { IDLE, SPAWNING, SPAWNED };

class RRProject : public rclcpp::Node
{
public:
    RRProject() : Node("rr_project")
    {
        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&RRProject::pose_callback, this, _1));

        spawner_timer = this->create_wall_timer(
            20ms, std::bind(&RRProject::spawn_timer_callback, this) // 20ms
        );

        turtle_spawn_service_client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        turtle_kill_service_client = this->create_client<turtlesim::srv::Kill>("/kill");

        dist = std::uniform_int_distribution<int>(1, 10);
        dist_theta = std::uniform_int_distribution<int>(-3, 3);

        target_pose = std::make_shared<turtlesim::msg::Pose>();

        while (!turtle_spawn_service_client->wait_for_service(1s) && !turtle_kill_service_client->wait_for_service(1s)) {
        
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting...");
                exit(-1);
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }
    }
private:

    //std::mt19937 rng(std::random_device());
    std::default_random_engine generator;
    std::uniform_int_distribution<int> dist;
    std::uniform_int_distribution<int> dist_theta;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::TimerBase::SharedPtr spawner_timer;

    turtlesim::msg::Pose::SharedPtr pose;
    turtlesim::msg::Pose::SharedPtr target_pose;

    geometry_msgs::msg::Twist speed;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr turtle_spawn_service_client;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr turtle_kill_service_client;

    bool target_exists = false;

    TurtleState state = IDLE;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr turtle_pose)
    {
        pose = turtle_pose;
    }

    void spawn_timer_callback()
    {
        speed.linear.x = 0.0;
        speed.angular.z = 0.0;

        if (state == IDLE) {
            spawn_turtle();
        }

        if (state == SPAWNED)
        {
            if (euclidean_distance(target_pose) > DISTANCE_TOLERANCE)
            {
                speed.linear.x = linear_vel(target_pose);

                if (abs(steering_angle(target_pose)) > ANGLE_TOLERANCE)
                {
                    speed.angular.z = angular_vel(target_pose);
                }
            }
            else
            {
                kill_turtle();
            }
        }

        velocity_publisher->publish(speed);
    }

    void spawn_turtle() 
    {
        std::cout << std::endl << std::endl;

        state = SPAWNING;

        target_pose->x = dist(generator);
        target_pose->y = dist(generator);
        target_pose->theta = dist_theta(generator);

        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();

        spawn_request->name = "target";
        spawn_request->x = target_pose->x;
        spawn_request->y = target_pose->y;
        spawn_request->theta = target_pose->theta;

        turtle_spawn_service_client->async_send_request(spawn_request, std::bind(&RRProject::spawn_turtle_response_callback, this, _1));
    }

    void spawn_turtle_response_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try 
        {
            future.get();
        } catch (const std::future_error& e) 
        {
            state = IDLE;
        }

        state = SPAWNED;
    }

    void kill_turtle() 
    {
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();

        kill_request->name = "target";

        turtle_kill_service_client->async_send_request(kill_request, std::bind(&RRProject::kill_turtle_response_callback, this, _1));
    }

    void kill_turtle_response_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        future.get();
        state = IDLE;
    }

    float euclidean_distance(turtlesim::msg::Pose::SharedPtr target_pose) 
    {
        return sqrt(pow(target_pose->x - pose->x, 2) + pow(target_pose->y - pose->y, 2));
    }

    float linear_vel(turtlesim::msg::Pose::SharedPtr target_pose, float speed = 1.5) 
    {
        return speed * euclidean_distance(target_pose);
    }

    float steering_angle(turtlesim::msg::Pose::SharedPtr target_pose)
    {
        return atan2(target_pose->y - pose->y, target_pose->x - pose->x);
    }

    float angular_vel(turtlesim::msg::Pose::SharedPtr target_pose, float speed = 6.0)
    {
        return speed * (steering_angle(target_pose) - pose->theta);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRProject>());
    rclcpp::shutdown();

    return 0;
}