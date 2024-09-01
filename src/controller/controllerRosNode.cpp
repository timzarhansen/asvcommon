//
// Created by tim-linux on 22.12.21.
//
#include "controllerOfBluerov2.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<controllerOfBluerov2>());
    rclcpp::shutdown();



    return (0);
}


