/**
 * @file eliko_driver.cpp 
 */
#include "rclcpp/rclcpp.hpp"
#include "../libs/eliko_driver.hpp"

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<eliko_driver>());
    rclcpp::shutdown();
    return 0;
}
