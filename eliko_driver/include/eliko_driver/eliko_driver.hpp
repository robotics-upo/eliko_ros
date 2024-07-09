/**
 * @file eliko_driver.hpp
 * @brief This file includes all the functions that obtain the data of the anchors, tags and live possitioning of the tags.
*/
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> 
#include <vector>
#include <sstream>
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include "eliko_data.h"
#include "eliko_messages/msg/distances_list.hpp"
#include "eliko_messages/msg/anchor_coords_list.hpp"
#include "eliko_messages/msg/tag_coords_list.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
/**
 * These rows store the values necessary to stablish communication with the server.
*/
#define DEFAULT_PORT 25025
#define DEFAULT_SERVER_IP "10.8.4.1"
#define POINTCLOUD_HEIGHT 1
#define SIZE 1000
#define DEFAULT_FRAME_ID "Eliko"//This row stores the value of the default frame id which will bw used to show the data in rviz2. 
/**
 * These next rows store the strings that will be used to identify the different types of messages
 * received from the radar.
*/
#define RR_L "RR_L"
#define ANCHOR_COORD "ANCHOR_COORD"
#define ANCHOR_COORD_E "ANCHOR_COORD_E"
#define COORD "COORD"
#define COORD_E "COORD_E"
#define NOT_UNDERSTAND "CANNOT_UNDERSTAND"
/**
 * These next define rows store the messages the user (driver) can send to the server to obtain
 * some data and some of the responses the server sends.
*/
#define GET_EULA_STATUS_COMMAND "$PEKIO,EULA,STATUS\r\n"
#define SET_ANCHOR "$PEKIO,SET_ANCHOR,"
#define ACCEPT_EULA_COMMAND "$PEKIO,EULA,ACCEPT\r\n"
#define GET_ANCHOR_COMMAND "$PEKIO,GET_ANCHORS\r\n"
#define GET_ANCHOR_COMMAND_E "$PEKIO,GET_ANCHORS_E\r\n"
#define GET_RRL_COMMAND "$PEKIO,SET_REPORT_LIST,RR_L\r\n"
#define GET_COORD_COMMAND "$PEKIO,SET_REPORT_LIST,COORD\r\n"
#define GET_COORD_E_COMMAND "$PEKIO,SET_REPORT_LIST,COORD_E\r\n"
#define GET_RRL_COORD_COMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD\r\n"
#define GET_RRL_COORD_E_COMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD_E\r\n"
#define GET_EVERYTHING_COMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD,COORD_E\r\n"

class ElikoDriver: public rclcpp::Node
{
  private:
  visualization_msgs::msg::Marker anchor_marker_;
  eliko_messages::msg::DistancesList all_distances_;
  eliko_messages::msg::AnchorCoordsList all_anchors_;
  eliko_messages::msg::TagCoordsList all_tags_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> tag_point_publishers_; 
  std::unordered_map<std::string,rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> anchor_tag_distance_publishers_;
  std::map<std::string,Coord>MapTags;
  /**
   * @brief Searches for a word inside a phrase and returns true or false depending if it founds it.
   * @param phrase The phrase that may have the word you are searching for.
   * @param word The word you are searching for.
  */
  bool containsWord(const std::string& phrase,const std::string& word){
    return phrase.find(word)!=std::string::npos;
  }

  /**
   * @brief Decides if the String given is an error or a warning.
   * @param phrase The String we want to see if its an error. 
  */
  bool isItAnError(const std::string& phrase){
    if(containsWord(phrase,"Error")||(containsWord(phrase,"Math")&&!containsWord(phrase,"Fallback")))
      return true;
    else
      return false;
  }

  /**
   * @brief Gets all the fields from a line.
   * @param line The line we want to split into the different fields of the command
   * @returns A string vector with all the fields from the command
  */
  std::vector<std::string> getWords(std::string line)
  {
    std::vector<std::string> sublines;
    std::vector<std::string> words;
    std::istringstream iss(line);
    std::string line_without_r;
    std::string word;
    // To delete the last \r value from the line
    while (std::getline(iss,line_without_r,'\r'))
      sublines.push_back(line_without_r);
    //To get all the fields in the line
    if(sublines.size()>0)
    {
      std::istringstream iss2(sublines[0]);
      while (std::getline(iss2,word,','))
      words.push_back(word);
    }
    return words;
  }

  /**
   * @brief To get all the Lines received from the server.
   * @param buffer The data received from the radar.
   * @returns A vector with all the lines that we have received.
  */
  std::vector<std::string> getLines(char buffer[1024])
  {
    std::vector<std::string> lines;
    std::istringstream iss(buffer);
    std::string line;            
    while (std::getline(iss,line))
      lines.push_back(line);
    return lines;
  }

  /**
   * @brief Fills the AnchorCoord struct fields with all of the data received.
   * @param words The vector of string with all of the data.
   * @returns The filled struct with coords of the anchors.
  */
  AnchorCoord fillAnchorCoord(std::vector<std::string>words)
  {
    AnchorCoord anchor;
    anchor.anchor_id=words[2];
    anchor.anchor_role=words[3][0];
    anchor.anchor_sn=words[4];
    if(words[5]!="")
    {
      anchor.anchor_position.x=stof(words[5]);
      anchor.anchor_position.y=stof(words[6]);
      anchor.anchor_position.z=stof(words[7]);
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
    anchor.connection_state=stoi(words[words.size()-1]);
    return anchor;
  }
  /**
   * @brief This function filld the distance Structure with the distance of each anchor to each one of the Tags.
   * @param distances The distance data received from the server.
  */
  void fillDistanceMessage(Rr_l distances,rclcpp::Clock clock)
  {
    eliko_messages::msg::Distances dist;
    if (distances.num_anchors>0)
    {
      all_distances_.header.frame_id=this->frame_id_;
      all_distances_.header.stamp=clock.now();
      all_distances_.anchor_distances.clear();
      for (int i=0;i<distances.num_anchors;i++)
      {
        dist.anchor_sn=distances.anchors[i].anchor_id;
        dist.distance=distances.anchors[i].distance;
        dist.tag_sn=distances.tag_sn;
        all_distances_.anchor_distances.push_back(dist);
      }
    }  
  }

  /**
   *  @brief This function searches for any publishers with the tag obtained when reading the coord messages received.
   * If it does not find any, it creates a new one.
   *  @param coord The coordinates data received from the server.  
  */
  void addPointPublisher(Coord coord)
  {
    std::string tag_sn=coord.tag_sn;
    if(tag_point_publishers_.count(tag_sn)==0)
    {
      auto publisher= this->create_publisher<geometry_msgs::msg::PointStamped>("Tag_Coords/T"+tag_sn,10);
      tag_point_publishers_[coord.tag_sn]=publisher;
    }
  }

  /**
   * @brief Fills the Rr_l struct with all of the data received.
   * @param words The vector string with all the data received.
   * @returns The filled struct with all the calculated distances from each anchor to the Tag.
  */
  Rr_l fillDistances(std::vector<std::string> words)
  {
    Rr_l distances;
    int j=0;//To count the actual number of anchors that take part in obtaining the distance from the Tag.
    /**
     * This three next lines are used because the number of fields of this message is variable.
     * This variation depends on the number of anchors that participate in this particular message.
    */
    int message_length=words.size();
    int num_anchors_related_messages=message_length-6;// Here we remove all of the fields that are not related to the anchors.
    int num_anchors=(num_anchors_related_messages/3);//Here we obtain the number of anchors that are detecting the Tag.
    // We divide the total Anchor fields between 3 because there are 3 fields per Anchor
    distances.seq_number=stoi(words[2]);
    distances.tag_sn=words[3];
    for (int i = 0; i < num_anchors; i++)
    {
      //Because sometimes the server sends more anchor fields that the ones related to the tag.
      if(words[4+(i*2)]!="0x000000")
      {
        DistanceToAnchor distance;
        ErrorFlags error;
        distance.anchor_id=words[4+(i*2)];
        distance.distance=stoi(words[5+(i*2)]);
        error.flag=words[5+(num_anchors*2)+i];
        j++;
        distances.anchors.push_back(distance);
        distances.flags.push_back(error);

      }  
    }
    distances.num_anchors=j;
    distances.timestamp=words[4+(num_anchors*2)];
    distances.tagError.flag=words[message_length-1];
    return distances;
  }
  void fillEachDistanceMessage(Rr_l distances)
  {
    std::string tag_sn=distances.tag_sn;
    for(int i=0; i<distances.num_anchors;i++){
      std::string anchor_id=distances.anchors.at(i).anchor_id;
      std::string address="Distance/A"+anchor_id+"/T"+tag_sn;
      if(anchor_tag_distance_publishers_.size()==0||anchor_tag_distance_publishers_.count(address)==0){
        auto publisher =this->create_publisher<std_msgs::msg::Int32>(address,10);
        anchor_tag_distance_publishers_[address]=publisher;
      }
      std_msgs::msg::Int32 msg;
      msg.data=distances.anchors[i].distance;
      if(distances.flags[i].flag=="0x00")
        anchor_tag_distance_publishers_[address]->publish(msg);
    }
  }
   

  /**
   * @brief This function fills the Tag message that will be sent to the user.
   * @param message The message with all of the tags coordinates.
  */
  void AddTagData(Coord message)
  {
    /**
     * These next lines are used to fill the message with all of the detected Tag coordinates.
     */
    MapTags[message.tag_sn].seq_number=message.seq_number;
    MapTags[message.tag_sn].tag_coords.x=message.tag_coords.x;
    MapTags[message.tag_sn].tag_coords.y=message.tag_coords.y;
    MapTags[message.tag_sn].tag_coords.z=message.tag_coords.z;
    MapTags[message.tag_sn].info=message.info;
    MapTags[message.tag_sn].timestamp=message.timestamp;
  }

  /**
   * @brief This function fills the anchor message that will be sent to the user.
   * @param message The message with all of the anchor data.
  */
  void fillAnchorMessage(AnchorCoord message,rclcpp::Clock clock)
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
    all_anchors_.header.stamp=clock.now();
    all_anchors_.anchor_coords.push_back(anchor);
  }

  /**
   * @brief Creates all the Marker messages that will be sent to rviz to show the Anchor Position.
   * @param anchors The anchor Struct message that will be used to fill the anchor possition messages.
   * @param clock 
  */
  void fillMarkerMessage(AnchorCoord anchors,rclcpp::Clock clock)
  {
    //To only show the connected and configured anchors in Rviz
    if(anchors.anchor_id!="NotConfigured"&&anchors.connection_state!=0) 
    {
      anchor_marker_.header.frame_id=this->frame_id_;
      anchor_marker_.header.stamp=clock.now();
      anchor_marker_.id=std::stoi(anchors.anchor_id,nullptr,16);
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
  /**
   * @brief Takes all the Tags detected by the eliko sensors and stores in the all_tags_ variables.
   * @param clock
   */
  void fillAllTags(rclcpp::Clock clock)
  {
    eliko_messages::msg::TagCoords tag;
    for (const auto& item:MapTags)
    {
      tag.tag_sn=item.first;
      tag.info=item.second.info;
      tag.x_coord=item.second.tag_coords.x;
      tag.y_coord=item.second.tag_coords.y;
      tag.z_coord=item.second.tag_coords.z;
      tag.seq_number=item.second.seq_number;
      tag.timestamp=item.second.timestamp;
      all_tags_.tag_coords.push_back(tag);
    }
    all_tags_.header.frame_id=this->frame_id_;
    all_tags_.header.stamp=clock.now(); 
  }
  /**
   * @brief Fills the Coord struct with all the data received.
   * @param words The vector string with all the data.
   * @returns The filled struct with the calculated coords for the Tag.
  */
  Coord fillCoords(std::vector<std::string> words)
  {
    Coord coordinates;
    if (words.size() >= 9)
    {
      coordinates.seq_number=stoi(words[2]);
      coordinates.tag_sn=words[3];
      if(words[4]!="")
      {
        coordinates.tag_coords.x=stof(words[4]);
        coordinates.tag_coords.y=stof(words[5]);
        coordinates.tag_coords.z=stof(words[6]);
        coordinates.info=words[7];
        coordinates.timestamp=stoi(words[8]);
      }
    }
    return coordinates;
  }

  /**
   * @brief Sends the message to obtain the Anchor Coords to the server, obtains all the data sent back by the server and stores them in a struct.
   * @param client_socket The Socket where the client is connected.
   * @param node  The Node created by the user.
  */
  void getAnchorCoords(int client_socket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock)
  {
    auto publisher_anchor_message=node->create_publisher<eliko_messages::msg::AnchorCoordsList>("AnchorCoords",10);
    auto publisher_anchor=node->create_publisher<visualization_msgs::msg::Marker>("MarkerCloudAnchors",10);
    send(client_socket,GET_ANCHOR_COMMAND,sizeof(GET_ANCHOR_COMMAND),0);
    int bytes_read;
    char buffer[1024]={0};
    while((bytes_read=recv(client_socket,buffer,sizeof(buffer),0))>0)
    {
      std::vector<std::string> lines;
      lines=getLines(buffer);
      if(lines.size()>0)
      {
        for (auto& l:lines)
        {
          std::vector<std::string>words;
          words=getWords(l);
          if(words.size()>1)
          {
            if(words[1]==ANCHOR_COORD)
            {
 
              AnchorCoord anchor;
              anchor=fillAnchorCoord(words);
              fillMarkerMessage(anchor,clock);
              fillAnchorMessage(anchor,clock);
              publisher_anchor->publish(anchor_marker_);
            }
            else if (words[1]=="EOF")
            {
              publisher_anchor_message->publish(all_anchors_);
              return;
            }
          }
        }
      }
    }     
  }

  /**
   * @brief Sends the message to receive the different information from the Tag and stores the response in a struct.
   * Once all the data is stored. it cleans the buffer.
   * @param client_socket The socket where the conversation is maintained.
   * @param node The driver node to create the different publishers. 
   * @param clock 
  */
  void setReportList(int client_socket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock)
  {
    auto publisherTagCoords=node->create_publisher<eliko_messages::msg::TagCoordsList>("TagCoords",10);
    auto publisher_distance=node->create_publisher<eliko_messages::msg::DistancesList>("Distances",10);
    send(client_socket,GET_RRL_COORD_COMMAND,sizeof(GET_RRL_COORD_COMMAND),0);
    char buffer[1024]={0};
    int bytes_read;
    //To have different sequence numbers for each tag; 
    std::map<std::string, int> seq_numbers;
    while((bytes_read=recv(client_socket,buffer,sizeof(buffer),0))>0)
    {
      /**
        * To show the coordinates of the anchors in the view
      */
      bool NewCoords=false;
      std::vector<std::string> lines;
      lines=getLines(buffer);
      if(lines.size()>0)
      {
        for (auto& l:lines)
        {
          std::vector<std::string>words;
          words=getWords(l);
          if(words.size()>1)
          {
            if(words[1]==RR_L)
            {
              //std::cout<<"Distances:"<<l<<std::endl;

              Rr_l tag_distances;

              tag_distances=fillDistances(words);
              fillDistanceMessage(tag_distances,clock);
              //to see if the tag has been seen before on the execution.
              if(seq_numbers.count(tag_distances.tag_sn) == 0 || 
                 (tag_distances.seq_number - seq_numbers[tag_distances.tag_sn] > 0 && 
                  tag_distances.seq_number - seq_numbers[tag_distances.tag_sn] < 3) ||
                 (tag_distances.seq_number < 3 && seq_numbers[tag_distances.tag_sn] > 250)
                 ){
                seq_numbers[tag_distances.tag_sn]=tag_distances.seq_number;
                //std::cout<<"Last Seq Number: "<<seq_numbers[tag_distances.tag_sn]<<std::endl;
                fillEachDistanceMessage(tag_distances);
                publisher_distance->publish(all_distances_);
              }
            }
            else if(words[1]==COORD)
            {
              NewCoords=true;
              /**
               * Here, we obtain the data received from the server where the identifier is equal to "COORD".
               * This message sends the possition calculated for a tag. It is asynchronous and is sent everytime a new position has been calculated.
              */
              Coord coordinates;
              coordinates=fillCoords(words);
              if(!isItAnError(coordinates.info)) //To see if the string received is an error or a warning sent by the Server
              {
                /**
                * This lines create the necessary elements to fill the point_msg that we will use to show the Tags on Rviz 
                */
                AddTagData(coordinates);
                addPointPublisher(coordinates);  
                
              }
            }
            else if(words[1]==NOT_UNDERSTAND)
            {
              /**
               * In case the message the driver sent is not supported or unrecognized
              */
              std::cout<<"Message not recognized/supported"<<std::endl;
              return;
            }
            
          }
        }
        if(NewCoords==true)
        {
          fillAllTags(clock);
          publisherTagCoords->publish(all_tags_);
          for( const auto& item:all_tags_.tag_coords){
            auto point_msg =std::make_shared<geometry_msgs::msg::PointStamped>();
            point_msg->header=std_msgs::msg::Header();
            point_msg->header.frame_id=this->frame_id_;
            point_msg->header.stamp=clock.now();
            point_msg->point.x=item.x_coord;
            point_msg->point.y=item.y_coord;
            point_msg->point.z=item.z_coord;
            tag_point_publishers_[item.tag_sn]->publish(*point_msg); 
          }
          all_tags_.tag_coords.clear();
        }      
      }
    }
    return;
  }

  /**
   * @brief To Check if the EULA has been accepted and, if it has not been accepted, accept it.
   * @param client_socket The socket where the client is having the conversation with the server.
  */
  void EULAStatus(int client_socket)
  {
    int bytes_read;
    char buffer[1024];
    send(client_socket,GET_EULA_STATUS_COMMAND,sizeof(GET_EULA_STATUS_COMMAND),0);
    while((bytes_read=recv(client_socket,buffer,sizeof(buffer),0))>0)
    {
      std::istringstream iss(buffer);
      std::vector<std::string>words;
      std::string word;
      while (std::getline(iss,word,','))
        words.push_back(word);
      if(words[1]=="NOT_GOOD")
        std::cout<<"There has been a problem"<<std::endl;
      else if(words[2]=="NOT_ACCEPTED")
      {
        send(client_socket,ACCEPT_EULA_COMMAND,sizeof(ACCEPT_EULA_COMMAND),0);
        EULAStatus(client_socket);
      }
      else
      {
        std::cout<<"EULA Accepted"<<std::endl;
        return;
      }
    }
  }
  
  public:
  std::string frame_id_;
  std::string server_ip_;
  int server_port_;

  /**
   * @brief eliko_driver Node. Used to try the driver.
  */
  ElikoDriver():Node("eliko_driver")
  {
    auto start=std::chrono::high_resolution_clock::now();
    std::chrono::seconds duration(14);
    auto node=rclcpp::Node::make_shared("publisherData");
    rclcpp::Clock clock;
    this->declare_parameter("serverIP",DEFAULT_SERVER_IP);
    this->declare_parameter("serverPort",DEFAULT_PORT);
    this->declare_parameter("frameID",DEFAULT_FRAME_ID);
    this->frame_id_=this->get_parameter("frameID").as_string();
    this->server_ip_=this->get_parameter("serverIP").as_string();
    this->server_port_=this->get_parameter("serverPort").as_int();
    int client_socket= socket(AF_INET,SOCK_STREAM,0);
    if(client_socket==-1)
      std::cerr<<"Socket creation error"<<std::endl;
    else
    {
      RCLCPP_DEBUG(node->get_logger(),"Socket Creado");
      sockaddr_in serverAddress;
      serverAddress.sin_family=AF_INET;
      serverAddress.sin_port=htons(DEFAULT_PORT);
      serverAddress.sin_addr.s_addr=inet_addr(DEFAULT_SERVER_IP);
      if (connect(client_socket,(struct sockaddr*)&serverAddress, sizeof(serverAddress))==-1)
      {
        std::cerr<<"Problem trying to connect to the server"<<std::endl;
        std::cout<<"Error connecting to the server"<<std::endl;
        RCLCPP_DEBUG(node->get_logger(),"Problema conectandome");
        close(client_socket);
      }
      else
      {
        RCLCPP_DEBUG(node->get_logger(),"Conectado");
        rclcpp::Clock clock;
        EULAStatus(client_socket);
        while (std::chrono::high_resolution_clock::now()- start<duration)
        {
          getAnchorCoords(client_socket,node,clock);
        } 
        setReportList(client_socket,node,clock);
      }    
    }
  }
};