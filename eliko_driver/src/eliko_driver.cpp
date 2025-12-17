/**
 * @file eliko_driver.cpp 
 */
#include "eliko_driver/eliko_driver.hpp" 

ElikoDriver::ElikoDriver():Node("eliko_driver"), client_socket_(-1), keep_running_(true)
{
  // Init Publishers
  pub_anchor_list_ = this->create_publisher<eliko_messages::msg::AnchorCoordsList>("AnchorCoords", 10);
  pub_anchor_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("MarkerCloudAnchors", rclcpp::QoS(10).transient_local());
  pub_tag_list_ = this->create_publisher<eliko_messages::msg::TagCoordsList>("TagCoords", 10);
  pub_distances_ = this->create_publisher<eliko_messages::msg::DistancesList>("Distances", 10);

  // Init Params
  this->declare_parameter("serverIP", DEFAULT_SERVER_IP);
  this->declare_parameter("serverPort", DEFAULT_PORT);
  this->declare_parameter("frameID", DEFAULT_FRAME_ID);
  
  this->frame_id_ = this->get_parameter("frameID").as_string();
  this->server_ip_ = this->get_parameter("serverIP").as_string();
  this->server_port_ = this->get_parameter("serverPort").as_int();

  RCLCPP_INFO(get_logger(), "Starting Eliko Driver for %s:%d", server_ip_.c_str(), server_port_);

  // Start thread immediately, connection handling is inside
  tcp_thread_ = std::thread(&ElikoDriver::runDriver, this);

  // Check every 1 second
  timeout_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ElikoDriver::checkTimeout, this));
}

ElikoDriver::~ElikoDriver() {
  keep_running_ = false;
  // Shutdown socket to break recv in thread
  if (client_socket_ != -1) {
    shutdown(client_socket_, SHUT_RDWR); 
    close(client_socket_);
  }
  if (tcp_thread_.joinable()) {
    tcp_thread_.join();
  }
}

float ElikoDriver::safeStof(const std::string& s) {
  try { return std::stof(s); } catch (...) { return 0.0f; }
}

int ElikoDriver::safeStoi(const std::string& s, int base) {
  try { return std::stoi(s, nullptr, base); } catch (...) { return 0; }
}

bool ElikoDriver::containsWord(const std::string& phrase,const std::string& word){
  return phrase.find(word)!=std::string::npos;
}

bool ElikoDriver::isItAnError(const std::string& phrase){
  if(containsWord(phrase,"Error")||(containsWord(phrase,"Math")&&!containsWord(phrase,"Fallback")))
    return true;
  else
    return false;
}

std::vector<std::string> ElikoDriver::getWords(std::string line)
{
  std::vector<std::string> words;
  std::string word;
  
  if (!line.empty() && line.back() == '\r') {
      line.pop_back();
  }
  std::istringstream iss(line);
  while (std::getline(iss,word,','))
    words.push_back(word);
  return words;
}

AnchorCoord ElikoDriver::fillAnchorCoord(const std::vector<std::string>& words)
{
  AnchorCoord anchor;
  if (words.size() < 10) return anchor;

  anchor.anchor_id=words[2];
  anchor.anchor_role = words[3].empty() ? ' ' : words[3][0];
  anchor.anchor_sn=words[4];
  if(words[5]!="")
  {
    anchor.anchor_position.x=safeStof(words[5]);
    anchor.anchor_position.y=safeStof(words[6]);
    anchor.anchor_position.z=safeStof(words[7]);
  }
  else
  {
    anchor.anchor_id="NotConfigured";
    anchor.anchor_position.x=0;
    anchor.anchor_position.y=0;
    anchor.anchor_position.z=0;
  }
  anchor.last_connection_established=words[8];
  anchor.last_connection_lost=words[9];
  anchor.connection_state=safeStoi(words[words.size()-1]);
  return anchor;
}

void ElikoDriver::fillDistanceMessage(const Rr_l& distances, rclcpp::Time now)
{
  eliko_messages::msg::Distances dist;
  if (distances.num_anchors>0)
  {
    all_distances_.header.frame_id=this->frame_id_;
    all_distances_.header.stamp=now;
    all_distances_.anchor_distances.clear();
    for (int i=0;i<distances.num_anchors;i++)
    {
      dist.anchor_sn=distances.anchors[i].anchor_id;
      dist.distance=static_cast<float>(distances.anchors[i].distance);
      dist.tag_sn=distances.tag_sn;
      all_distances_.anchor_distances.push_back(dist);
    }
  }  
}

void ElikoDriver::addPointPublisher(const std::string& tag_sn)
{
  if(tag_point_publishers_.count(tag_sn)==0)
  {
    auto publisher= this->create_publisher<geometry_msgs::msg::PointStamped>("Tag_Coords/T"+tag_sn,10);
    tag_point_publishers_[tag_sn]=publisher;
  }
}

Rr_l ElikoDriver::fillDistances(const std::vector<std::string>& words)
{
  Rr_l distances;
  int j=0;
  
  if (words.size() < 4) return distances;

  int message_length=words.size();
  int num_anchors_related_messages=message_length-6;
  int num_anchors=(num_anchors_related_messages/3);
  
  distances.seq_number=safeStoi(words[2]);
  distances.tag_sn=words[3];
  
  for (int i = 0; i < num_anchors; i++)
  {
    int idx_base = 4 + (i*2);
    if (idx_base + 1 >= (int)words.size()) break; 

    if(words[idx_base]!="0x000000")
    {
      DistanceToAnchor distance;
      ErrorFlags error;
      distance.anchor_id=words[idx_base];
      distance.distance=safeStoi(words[idx_base+1]);
      
      int err_idx = 5+(num_anchors*2)+i;
      if(err_idx < (int)words.size()) error.flag=words[err_idx];
      
      j++;
      distances.anchors.push_back(distance);
      distances.flags.push_back(error);
    }  
  }
  distances.num_anchors=j;
  return distances;
}

void ElikoDriver::fillEachDistanceMessage(const Rr_l& distances)
{
  std::string tag_sn=distances.tag_sn;
  for(int i=0; i<distances.num_anchors;i++){
    std::string anchor_id=distances.anchors.at(i).anchor_id;
    std::string address="Distance/A"+anchor_id+"/T"+tag_sn;
    if(anchor_tag_distance_publishers_.count(address)==0){
      auto publisher =this->create_publisher<std_msgs::msg::Int32>(address,10);
      anchor_tag_distance_publishers_[address]=publisher;
    }
    std_msgs::msg::Int32 msg;
    msg.data=distances.anchors[i].distance;
    
    if(distances.flags.size() > (size_t)i && distances.flags[i].flag=="0x00")
      anchor_tag_distance_publishers_[address]->publish(msg);
  }
}

void ElikoDriver::fillAnchorMessage(const AnchorCoord& message,rclcpp::Time now)
{
  eliko_messages::msg::AnchorCoords anchor;
  anchor.anchor_id=message.anchor_id;
  anchor.anchor_sn=message.anchor_sn;
  anchor.connection_state=message.connection_state;
  anchor.role=message.anchor_role;
  anchor.x_coord=message.anchor_position.x;
  anchor.y_coord=message.anchor_position.y;
  anchor.z_coord=message.anchor_position.z;
  anchor.last_connection_established=message.last_connection_established;
  anchor.last_connection_lost=message.last_connection_lost;
  all_anchors_.header.frame_id=this->frame_id_;
  all_anchors_.header.stamp=now;
  all_anchors_.anchor_coords.push_back(anchor);
}

void ElikoDriver::fillMarkerMessage(const AnchorCoord& anchors,rclcpp::Time now)
{
  if(anchors.anchor_id!="NotConfigured"&&anchors.connection_state!=0) 
  {
    anchor_marker_.header.frame_id=this->frame_id_;
    anchor_marker_.header.stamp=now;
    anchor_marker_.id=safeStoi(anchors.anchor_id, 16);
    anchor_marker_.ns=anchors.anchor_sn;
    anchor_marker_.type=visualization_msgs::msg::Marker::CYLINDER;
    anchor_marker_.pose.position.x=anchors.anchor_position.x;
    anchor_marker_.pose.position.y=anchors.anchor_position.y;
    anchor_marker_.pose.position.z=anchors.anchor_position.z;
    anchor_marker_.scale.x=0.2;
    anchor_marker_.scale.y=0.2;
    anchor_marker_.scale.z=0.2;
    anchor_marker_.color.a=1.0;
    anchor_marker_.color.r=1.0;
    anchor_marker_.color.g=0.0;
    anchor_marker_.color.b=0.0;
  } 
}

Coord ElikoDriver::fillCoords(const std::vector<std::string>& words)
{
  Coord coordinates;
  coordinates.seq_number = 0;
  coordinates.tag_coords = {0,0,0};
  
  if (words.size() >= 9)
  {
    coordinates.seq_number=safeStoi(words[2]);
    coordinates.tag_sn=words[3];
    if(words[4]!="")
    {
      coordinates.tag_coords.x=safeStof(words[4]);
      coordinates.tag_coords.y=safeStof(words[5]);
      coordinates.tag_coords.z=safeStof(words[6]);
      coordinates.info=words[7];
      coordinates.timestamp=safeStof(words[8]);
    }
  }
  return coordinates;
}

void ElikoDriver::sendCommand(const char* cmd) {
  if (client_socket_ != -1) {
      send(client_socket_, cmd, strlen(cmd), 0);
  }
}

std::vector<std::string> ElikoDriver::extractLinesFromBuffer() {
  std::vector<std::string> lines;
  size_t pos;
  while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
    std::string line = rx_buffer_.substr(0, pos);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      lines.push_back(line);
    }
    rx_buffer_.erase(0, pos + 1);
  }
  return lines;
}

void ElikoDriver::runDriver() {
  RCLCPP_INFO(get_logger(), "Driver Thread Started.");
    
  while (keep_running_ && rclcpp::ok()) {
    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if(client_socket_ == -1) {
      RCLCPP_ERROR(get_logger(), "Socket creation error. Retrying in 2s...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(this->server_port_);
    serverAddress.sin_addr.s_addr = inet_addr(this->server_ip_.c_str());

    RCLCPP_INFO(get_logger(), "Connecting to Eliko Server at %s:%d...", server_ip_.c_str(), server_port_);

    int flags = fcntl(client_socket_, F_GETFL, 0);
    fcntl(client_socket_, F_SETFL, flags | O_NONBLOCK);

    bool connected = false;
    int res = connect(client_socket_, (struct sockaddr*)&serverAddress, sizeof(serverAddress));

    if (res < 0) {
      if (errno == EINPROGRESS) {
        fd_set wait_set;
        FD_ZERO(&wait_set);
        FD_SET(client_socket_, &wait_set);

        struct timeval timeout;
        timeout.tv_sec = 3; 
        timeout.tv_usec = 0;

        int select_res = select(client_socket_ + 1, NULL, &wait_set, NULL, &timeout);
        
        if (select_res > 0) {

          int so_error;
          socklen_t len = sizeof(so_error);
          getsockopt(client_socket_, SOL_SOCKET, SO_ERROR, &so_error, &len);
          if (so_error == 0) {
            connected = true;
          }
        } 
      }
    } else {
      connected = true;
    }

    fcntl(client_socket_, F_SETFL, flags);

    if (!connected) {
      RCLCPP_WARN(get_logger(), "Connection attempt timed out or failed. Retrying...");
      close(client_socket_);
      client_socket_ = -1;
      std::this_thread::sleep_for(std::chrono::seconds(2)); 
      continue;
    }

    RCLCPP_INFO(get_logger(), "Connected!");
    rx_buffer_.clear();
    
    char buffer[2048];
    bool eula_accepted = false;
    sendCommand(GET_EULA_STATUS_COMMAND);
        
    bool getting_anchors = false;

    // Receiving loop
    while (keep_running_ && rclcpp::ok()) {
      int bytes_read = recv(client_socket_, buffer, sizeof(buffer)-1, 0);
      if (bytes_read <= 0) {
        if (!rclcpp::ok() || !keep_running_ || (bytes_read == -1 && errno == EINTR)) {
          RCLCPP_INFO(get_logger(), "Shutting down...");
          break;
        }
        RCLCPP_INFO(get_logger(), "Connection lost (recv returned %d). Reconnecting...", bytes_read);
        break;
      }
      buffer[bytes_read] = '\0';
      rx_buffer_ += buffer;

      std::vector<std::string> lines = extractLinesFromBuffer();
      for (const auto& line : lines) {
        std::vector<std::string> words = getWords(line);
        if (words.empty()) continue;

        // EULA Logic
        if (!eula_accepted) {
          if (containsWord(line, "EULA") && containsWord(line, "NOT_ACCEPTED")) {
            RCLCPP_INFO(get_logger(), "Accepting EULA...");
            sendCommand(ACCEPT_EULA_COMMAND);
            sendCommand(GET_EULA_STATUS_COMMAND);
          } else if (containsWord(line, "EULA") && containsWord(line, "ACCEPTED")) {
            RCLCPP_INFO(get_logger(), "EULA Accepted. Getting anchors...");
            eula_accepted = true;
            sendCommand(GET_ANCHOR_COMMAND);
            getting_anchors = true;
          }
          continue;
        }

        // Anchor Logic
        if (getting_anchors) {
          if (words.size() > 1 && words[1] == ANCHOR_COORD) {
            AnchorCoord anchor = fillAnchorCoord(words);
            rclcpp::Time anchor_time = this->now();
            fillMarkerMessage(anchor, anchor_time);
            fillAnchorMessage(anchor, anchor_time);
            pub_anchor_marker_->publish(anchor_marker_);
          }
          else if (words.size() > 1 && words[1] == "EOF") {
            RCLCPP_INFO(get_logger(), "Anchors received. Switching to live data...");
            pub_anchor_list_->publish(all_anchors_);
            getting_anchors = false;
            sendCommand(GET_RRL_COORD_COMMAND);
          }
          continue;
        }

        // Live Data Logic (RR_L and COORD)
        rclcpp::Time now = this->now();
        if (words.size() > 1) {
          if (words[1] == RR_L) {
            Rr_l tag_distances = fillDistances(words);
            {
              std::lock_guard<std::mutex> lock(timeout_mutex_);
              std::string tag = tag_distances.tag_sn;
              if (tag_is_lost_[tag]) {
                RCLCPP_INFO(get_logger(), "Signal recovered from tag %s", tag.c_str());
                tag_is_lost_[tag] = false;
              }
              tag_last_seen_[tag] = now;
            }
            fillDistanceMessage(tag_distances, now);
            fillEachDistanceMessage(tag_distances);
            pub_distances_->publish(all_distances_);
          }
          
          else if (words[1] == COORD) {
            Coord coordinates = fillCoords(words);
            {
              std::lock_guard<std::mutex> lock(timeout_mutex_);
              std::string tag = coordinates.tag_sn;
              if (tag_is_lost_[tag]) {
                RCLCPP_INFO(get_logger(), "Signal recovered from tag %s", tag.c_str());
                tag_is_lost_[tag] = false;
              }
              tag_last_seen_[tag] = now;
            }
            
            if (!isItAnError(coordinates.info)) {
              
              all_tags_.header.frame_id = this->frame_id_;
              all_tags_.header.stamp = now;
              all_tags_.tag_coords.clear();
              
              eliko_messages::msg::TagCoords tag;
              tag.tag_sn = coordinates.tag_sn;
              tag.info = coordinates.info;
              tag.x_coord = coordinates.tag_coords.x;
              tag.y_coord = coordinates.tag_coords.y;
              tag.z_coord = coordinates.tag_coords.z;
              tag.seq_number = coordinates.seq_number;
              tag.timestamp = coordinates.timestamp;
              all_tags_.tag_coords.push_back(tag);
                            
              pub_tag_list_->publish(all_tags_);

              addPointPublisher(coordinates.tag_sn);
              if (tag_point_publishers_.count(coordinates.tag_sn)) {
                auto point_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
                point_msg->header.frame_id = this->frame_id_;
                point_msg->header.stamp = now;
                point_msg->point.x = coordinates.tag_coords.x;
                point_msg->point.y = coordinates.tag_coords.y;
                point_msg->point.z = coordinates.tag_coords.z;
                tag_point_publishers_[coordinates.tag_sn]->publish(*point_msg);
              }
            }
          }
        }
      } 
    } // End Inner Loop (Receiving)
        
    // Ensure socket is closed before retrying
    if(client_socket_ != -1) {
      close(client_socket_);
      client_socket_ = -1;
    }

  } // End Outer Loop (Connection)
    
  RCLCPP_INFO(get_logger(), "Driver Thread Stopping.");
}

void ElikoDriver::checkTimeout()
{
  std::lock_guard<std::mutex> lock(timeout_mutex_);
  rclcpp::Time now = this->now();
  
  for (auto& pair : tag_last_seen_) {
    std::string tag = pair.first;
    if (!tag_is_lost_[tag]) {
      double seconds_since_last = (now - pair.second).seconds();
      if (seconds_since_last > 2.0) {
        RCLCPP_WARN(get_logger(), "No data received from tag %s", tag.c_str());
        tag_is_lost_[tag] = true; 
      }
    }
  }
}

int main(int argc,char* argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ElikoDriver>());
  rclcpp::shutdown();
  return 0;
}