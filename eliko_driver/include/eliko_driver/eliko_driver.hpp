/**
 * @file eliko_driver.hpp
 * @brief This file includes all the functions that obtain the data of the anchors, tags and live possitioning of the tags.
*/
#ifndef ELIKO_DRIVER_HPP
#define ELIKO_DRIVER_HPP

#include <iostream>
#include <stdio.h>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> 
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>

#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "eliko_data.h"
#include "eliko_messages/msg/distances_list.hpp"
#include "eliko_messages/msg/anchor_coords_list.hpp"
#include "eliko_messages/msg/tag_coords_list.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// Constants
static const int DEFAULT_PORT = 25025;
static const char* DEFAULT_SERVER_IP = "10.8.4.1";
static const int POINTCLOUD_HEIGHT = 1;
static const int SIZE = 1000;
static const char* DEFAULT_FRAME_ID = "eliko"; 

// Message identifiers
static const char* RR_L = "RR_L";
static const char* ANCHOR_COORD = "ANCHOR_COORD";
static const char* COORD = "COORD";

// Commands
static const char* GET_EULA_STATUS_COMMAND = "$PEKIO,EULA,STATUS\r\n";
static const char* ACCEPT_EULA_COMMAND = "$PEKIO,EULA,ACCEPT\r\n";
static const char* GET_ANCHOR_COMMAND = "$PEKIO,GET_ANCHORS\r\n";
static const char* GET_RRL_COORD_COMMAND = "$PEKIO,SET_REPORT_LIST,RR_L,COORD\r\n";

class ElikoDriver: public rclcpp::Node
{

public:
  /**
   * @brief eliko_driver Node. Used to try the driver.
  */
  ElikoDriver();
  ~ElikoDriver();

private:
  visualization_msgs::msg::Marker anchor_marker_;
  eliko_messages::msg::DistancesList all_distances_;
  eliko_messages::msg::AnchorCoordsList all_anchors_;
  eliko_messages::msg::TagCoordsList all_tags_;

  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> tag_point_publishers_; 
  std::unordered_map<std::string,rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> anchor_tag_distance_publishers_;
  
  rclcpp::Publisher<eliko_messages::msg::AnchorCoordsList>::SharedPtr pub_anchor_list_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_anchor_marker_;
  rclcpp::Publisher<eliko_messages::msg::TagCoordsList>::SharedPtr pub_tag_list_;
  rclcpp::Publisher<eliko_messages::msg::DistancesList>::SharedPtr pub_distances_;

  int client_socket_;
  std::string server_ip_;
  int server_port_;
  std::string frame_id_;
  
  std::thread tcp_thread_;          
  std::atomic<bool> keep_running_;  
  std::string rx_buffer_;           

  // Timeout monitoring
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::map<std::string, rclcpp::Time> tag_last_seen_;
  std::map<std::string, bool> tag_is_lost_;
  std::mutex timeout_mutex_;
  void checkTimeout();

  // Helper functions
  float safeStof(const std::string& s);
  int safeStoi(const std::string& s, int base = 10);
  bool containsWord(const std::string& phrase,const std::string& word);
  bool isItAnError(const std::string& phrase);
  std::vector<std::string> getWords(std::string line);

  // Data processing functions
  AnchorCoord fillAnchorCoord(std::vector<std::string>words);
  void fillDistanceMessage(Rr_l distances,rclcpp::Time now);
  void addPointPublisher(std::string tag_sn);
  Rr_l fillDistances(std::vector<std::string> words);
  void fillEachDistanceMessage(Rr_l distances);
  void fillAnchorMessage(AnchorCoord message,rclcpp::Time now);
  void fillMarkerMessage(AnchorCoord anchors,rclcpp::Time now);
  Coord fillCoords(std::vector<std::string> words);

  // Communication functions
  void sendCommand(const char* cmd);
  std::vector<std::string> extractLinesFromBuffer();
  void runDriver();
};

#endif // ELIKO_DRIVER_HPP