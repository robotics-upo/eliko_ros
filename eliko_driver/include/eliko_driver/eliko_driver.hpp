/**
 * @file eliko_driver.hpp
 * @brief This file includes all the functions that obtain the data of the anchors, tags and live possitioning of the tags.
*/

#include <iostream>
#include <stdio.h>
#include <cstring>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rclcpp/rclcpp.hpp"
#include "eliko_data.h"
#include "eliko_messages/msg/distances_list.hpp"
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
 * These next define rows store the messages the user (driver) sends to the server to obtain
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
  //sensor_msgs::msg::PointCloud2 cloud_tags_;
  sensor_msgs::msg::PointCloud2 cloud_anchors_;
  eliko_messages::msg::DistancesList all_distances_;
  sensor_msgs::PointCloud2Modifier anchor_modifier_;
  //sensor_msgs::PointCloud2Modifier tag_modifier_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> tag_point_publishers_; 
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
    if(sublines.size()>0){
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
    //char  answer;
    if(words[5]!="")
    {
      anchor.anchor_position.x=stof(words[5]);
      anchor.anchor_position.y=stof(words[6]);
      anchor.anchor_position.z=stof(words[7]);
    }
    else
    {
      anchor.anchor_position.x=0;
      anchor.anchor_position.y=0;
      anchor.anchor_position.z=0;
    }

    anchor.last_connection_established=words[8];
    anchor.last_connection_lost=words[9];
    anchor.connection_state=stoi(words[words.size()-1]);
    return anchor;
  }
  void fillDistanceMessage(Rr_l distances,rclcpp::Clock clock)
  {
    for (int i=0;i<distances.num_anchors;i++)
    {
      all_distances_.header.frame_id=this->frame_id_;
      all_distances_.header.stamp=clock.now();
      all_distances_.anchor_distances[i].anchor_sn=distances.anchors[i].anchor_id;
      all_distances_.anchor_distances[i].distance=distances.anchors[i].distance;
      all_distances_.anchor_distances[i].tag_sn=distances.tag_sn;
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
    if(tag_point_publishers_.count(tag_sn)==0){
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
    for (int i = 0, j=0; i < num_anchors; i++)
    {
      
      if(words[4+(i*2)]!="0x000000")
      {
        distances.anchors[j].anchor_id=words[4+(i*2)];
        distances.anchors[j].distance=stoi(words[5+(i*2)]);
        distances.flags[j].flag=words[5+(num_anchors*2)+i];
        j++;
        distances.num_anchors=j;
      }
      
    }
    distances.timestamp=words[4+(num_anchors*2)];
    distances.tagError.flag=words[message_length-1];
    return distances;
  }

  /**
   * @brief Fills the Coord struct with all the data received.
   * @param words The vector string with all the data.
   * @returns The filled struct with the calculated coords for the Tag.
  */
  Coord fillCoords(std::vector<std::string> words)
  {
    Coord coordinates;
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
    return coordinates;
  }
  

  /**
   * @brief Puts the headers to all of the PointCloud messages that will be sent to rviz.
   * @param cloud_msg The pointcloud message that needs to be filled.
  */
  void fillCloudMessage(sensor_msgs::msg::PointCloud2 &cloud_msg,rclcpp::Clock clock)
  {
    cloud_msg.header=std_msgs::msg::Header();
    cloud_msg.header.frame_id=this->frame_id_;
    cloud_msg.header.stamp=clock.now();
    cloud_msg.is_dense=false;
    cloud_msg.is_bigendian=false;
    cloud_msg.height=POINTCLOUD_HEIGHT;
  }
  /**
   * @brief Sends the message to obtain the Anchor Coords to the server, obtains all the data sent back by the server and stores them in a struct.
   * @param client_socket The Socket where the client is connected.
   * @param node  The Node created by the user.
  */
  void getAnchorCoords(int client_socket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock)
  {
    auto publisher_anchor=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudAnchors",10);
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
          sensor_msgs::PointCloud2Iterator<float>pos_X(cloud_anchors_,"x");
          sensor_msgs::PointCloud2Iterator<float>pos_Y(cloud_anchors_,"y");
          sensor_msgs::PointCloud2Iterator<float>pos_Z(cloud_anchors_,"z");
          anchor_modifier_.resize(lines.size());
          std::vector<std::string>words;
          words=getWords(l);
          if(words.size()>1)
          {
            if(words[1]==ANCHOR_COORD)
            {
              fillCloudMessage(cloud_anchors_,clock);
              AnchorCoord anchor;
              anchor=fillAnchorCoord(words);
              //To fill up the pointCloud
              *pos_X=anchor.anchor_position.x;
              *pos_Y=anchor.anchor_position.y;
              *pos_Z=anchor.anchor_position.z;
              publisher_anchor->publish(cloud_anchors_);
            }
            else if (words[1]=="EOF")
              return;
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
  */
  void setReportList(int client_socket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock)
  {
    auto publisher_distance=node->create_publisher<eliko_messages::msg::DistancesList>("Distances",10);
    //auto publisher_tag=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudTags",10);
    send(client_socket,GET_RRL_COORD_COMMAND,sizeof(GET_RRL_COORD_COMMAND),0);
    char buffer[1024]={0};
    int bytes_read;
    while((bytes_read=recv(client_socket,buffer,sizeof(buffer),0))>0)
    {
      //fillCloudMessage(cloud_tags_,clock);
      std::vector<std::string> lines;
      lines=getLines(buffer);
      if(lines.size()>0)
      {
        //tag_modifier_.resize(lines.size());
        for (auto& l:lines)
        {
          /*
          sensor_msgs::PointCloud2Iterator<float>pos_XT(cloud_tags_,"x");
          sensor_msgs::PointCloud2Iterator<float>pos_YT(cloud_tags_,"y");
          sensor_msgs::PointCloud2Iterator<float>pos_ZT(cloud_tags_,"z");
          */
          std::vector<std::string>words;
          words=getWords(l);
          if(words.size()>1)
          {
            if(words[1]==RR_L)
            {
              Rr_l tag_distances;
              tag_distances=fillDistances(words);
              fillDistanceMessage(tag_distances,clock);
              publisher_distance->publish(all_distances_);
            }
            else if(words[1]==COORD)
            {
              /**
               * Here, we obtain the data received from the server where the identifier is equal to "COORD".
               * This message sends the possition calculated for a tag. It is asynchronous and is sent everytime a new position has been calculated.
              */
              Coord coordinates;
              coordinates=fillCoords(words);
              /**
               * This lines create the necessaty elements to fill the point_msg that we will use to show the Tags on Rviz
              */
              auto point_msg =std::make_shared<geometry_msgs::msg::PointStamped>();
              point_msg->header=std_msgs::msg::Header();
              point_msg->header.frame_id=this->frame_id_;
              point_msg->header.stamp=clock.now();
              point_msg->point.x=coordinates.tag_coords.x;
              point_msg->point.y=coordinates.tag_coords.y;
              point_msg->point.z=coordinates.tag_coords.z;
              addPointPublisher(coordinates);
              tag_point_publishers_[coordinates.tag_sn]->publish(*point_msg);
             /*
               if(words[4]!=""){
                *pos_XT=coordinates.tag_coords.x;
                *pos_YT=coordinates.tag_coords.y;
                *pos_ZT=coordinates.tag_coords.z;                
                publisher_tag->publish(cloud_tags_);
              }*/
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
  ElikoDriver():Node("eliko_driver"),anchor_modifier_(cloud_anchors_)//,tag_modifier_(cloud_tags_)
  {
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
        //To accept the EULA and receive all of the messages from eliko.
        EULAStatus(client_socket);  
        //Set fields and size of every PointCloud
        //AnchorCloud
        anchor_modifier_.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        anchor_modifier_.reserve(SIZE);
        anchor_modifier_.clear();
        getAnchorCoords(client_socket,node,clock);
        //Tag Cloud
     /*   tag_modifier_.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        tag_modifier_.reserve(SIZE);
        tag_modifier_.clear();*/
        setReportList(client_socket,node,clock);
      }    
    }
  }
};