/**
 * @file eliko_driver.cpp 
 */
#include "rclcpp/rclcpp.hpp"
#include "../include/eliko_driver/eliko_driver.hpp"

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Eliko_driver>());
    rclcpp::shutdown();
    return 0;
}
