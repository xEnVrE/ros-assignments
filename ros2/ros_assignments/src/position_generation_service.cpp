#include "rclcpp/rclcpp.hpp"
#include "ros_assignments/srv/position_generator.hpp"

#include <cstdlib>
#include <random>

using PositionGenerator = ros_assignments::srv::PositionGenerator;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class PositionGeneratorService : public rclcpp::Node
{
public:
    PositionGeneratorService() :
        Node("position_generation_service"),
        generator_(std::mt19937(rd_())),
        distribution_(std::uniform_real_distribution<double>(-8.0, 8.0))
    {
        service_ = this->create_service<PositionGenerator>
        (
            "position_generation_service",
            std::bind(&PositionGeneratorService::callback, this, _1, _2, _3)
        );
    }

private:

    void callback
    (
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<PositionGenerator::Request> request,
        const std::shared_ptr<PositionGenerator::Response> response
    )
    {
        (void)request_header;
        (void)request;
        response->x = distribution_(generator_);
        response->y = distribution_(generator_);
    }

    rclcpp::Service<PositionGenerator>::SharedPtr service_;

    std::random_device rd_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> distribution_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionGeneratorService>());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
