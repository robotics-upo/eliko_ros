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
#include "eliko_messages/msg/distances.hpp"
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
    sensor_msgs::msg::PointCloud2 cloud_tags_;
    sensor_msgs::msg::PointCloud2 cloud_anchors_;
    eliko_messages::msg::Distances distance_to_tags_;
    sensor_msgs::PointCloud2Modifier anchor_modifier_;
    sensor_msgs::PointCloud2Modifier tag_modifier_;
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
            std::istringstream iss(buffer);
            std::string line;            
            while (std::getline(iss,line))
                lines.push_back(line);
            if(lines.size()>0)
            {
                for (auto& l:lines)
                {
                    sensor_msgs::PointCloud2Iterator<float>pos_X(cloud_anchors_,"x");
                    sensor_msgs::PointCloud2Iterator<float>pos_Y(cloud_anchors_,"y");
                    sensor_msgs::PointCloud2Iterator<float>pos_Z(cloud_anchors_,"z");
                    anchor_modifier_.resize(lines.size());
                    std::istringstream ss(l);
                    std::vector<std::string>words;
                    std::string word;
                    
                    while(std::getline(ss,word,','))
                    {
                        words.push_back(word);
                    }
                    if(words.size()>1)
                    {
                        if(words[1]==ANCHOR_COORD)
                        {
                            fillCloudMessage(cloud_anchors_,clock);
                            //To eliminate the '/r' from the end of the line    
                            std::istringstream ss2(words[10]);
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
                                anchor.anchor_position.x=0;
                                anchor.anchor_position.y=0;
                                anchor.anchor_position.z=0;
                            }
                        
                            anchor.last_connection_established=words[8];
                            anchor.last_connection_lost=words[9];
                            std::vector<std::string>last_values;
                            std::string connection_state;
                            while(std::getline(ss2,word,'\r'))
                            {
                                last_values.push_back(word);
                            }
                            connection_state=last_values[0];
                            anchor.connection_state=stoi(connection_state);
                            //To fill up the pointCloud
                            *pos_X=anchor.anchor_position.x;
                            *pos_Y=anchor.anchor_position.y;
                            *pos_Z=anchor.anchor_position.z;
                            publisher_anchor->publish(cloud_anchors_);
                        }
                        else if (words[1]=="EOF\r")
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
        auto publisher_distance=node->create_publisher<eliko_messages::msg::Distances>("Distances",10);
        auto publisher_tag=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudTags",10);
        send(client_socket,GET_RRL_COORD_COMMAND,sizeof(GET_RRL_COORD_COMMAND),0);
        char buffer[1024]={0};
        int bytes_read;
        while((bytes_read=recv(client_socket,buffer,sizeof(buffer),0))>0)
        {
            fillCloudMessage(cloud_tags_,clock);
            std::vector<std::string> lines;
            std::istringstream iss(buffer);
            std::string line;
            while (std::getline(iss,line))
                lines.push_back(line);
            if(lines.size()>0)
            {
                tag_modifier_.resize(lines.size());
                for (auto& l:lines)
                {
                    sensor_msgs::PointCloud2Iterator<float>pos_XT(cloud_tags_,"x");
                    sensor_msgs::PointCloud2Iterator<float>pos_YT(cloud_tags_,"y");
                    sensor_msgs::PointCloud2Iterator<float>pos_ZT(cloud_tags_,"z");
                    std::istringstream ss(l);
                    std::vector<std::string>words;
                    std::string word;
                    while(std::getline(ss,word,','))
                       words.push_back(word);
                    if(words.size()>1)
                    {
                        if(words[1]==RR_L)
                        {
                            /**
                             * Here we get all the data received from the server under the identifier "RR_L" which
                             * sends us the distances between the Tag and all the anchors. 
                            */
                            Rr_l distances; 
                            /**
                             * This three next lines are used because the number of fields of this message is variable.
                             * This variation depends on the number of anchors that participate in this particular message.
                            */
                            int message_length=words.size()-1;
                            int num_anchors_related_messages=message_length-6;// Here we remove all of the fields that are not related to the anchors.
                            int num_anchors=(num_anchors_related_messages/3);//Here we obtain the number of anchors that are detecting the Tag.
                            // We divide the total Anchor fields between 3 because there are 3 fields per Anchor
                            distances.seq_number=stoi(words[2]);
                            distances.tag_sn=words[3];
                            for (int i = 0; i < num_anchors; i++)
                            {
                                distances.anchors[i].anchor_id=words[4+(i*2)];
                                distances.anchors[i].distance=stoi(words[5+(i*2)]);
                                distance_to_tags_.header.frame_id=this->frame_id_;
                                distance_to_tags_.header.stamp=clock.now();
                                distance_to_tags_.anchor_sn=distances.anchors[i].anchor_id;
                                distance_to_tags_.distance=distances.anchors[i].distance;
                                distance_to_tags_.tag_sn=distances.tag_sn;
                                publisher_distance->publish(distance_to_tags_);
                            }
                            distances.timestamp=words[6+(num_anchors*2)];
                            for(int i=0;i<num_anchors; i++)
                            {
                                distances.flags[i].flag=words[7+(num_anchors*2)+i];
                            }
                            std::istringstream ss2(words[message_length-1]);
                            std::vector<std::string> last_values;
                            while (std::getline(ss2,word,'\r'))
                                /** 
                                 * We do this because every message ends with "\r\n" and when we split all of the messages,
                                 * we get rid of '\n' but not '\r' which appears at the end of the last field.
                                */  
                                last_values.push_back(word);
                            distances.tagError.flag=last_values[0];
                            std::cout<<distances.tag_sn<<std::endl;
                        }
                        else if(words[1]==COORD)
                        {
                            /**
                             * Here, we obtain the data received from the server where the identifier is equal to "COORD".
                             * This message sends the possition calculated for a tag. It is asynchronous and is sent everytime a new position has been calculated.
                            */
                            Coord coordinates;
                            coordinates.seq_number=stoi(words[2]);
                            coordinates.tag_sn=words[3];
                            if(words[4]!="")
                            {
                                coordinates.tag_coords.x=stof(words[4]);
                                coordinates.tag_coords.y=stof(words[5]);
                                coordinates.tag_coords.z=stof(words[6]);
                                *pos_XT=coordinates.tag_coords.x;
                                *pos_YT=coordinates.tag_coords.y;
                                *pos_ZT=coordinates.tag_coords.z;
                                std::vector<std::string>last_values;
                                std::string timestamp;
                                std::istringstream ss2(words[words.size()-1]);
                                coordinates.info=words[7];
                                while(std::getline(ss2,word,'\r'))
                                {
                                    last_values.push_back(word);
                                }
                                timestamp=last_values[0];
                                coordinates.timestamp=stoi(timestamp);
                                publisher_tag->publish(cloud_tags_);
                                std::cout<<"X:"<<coordinates.tag_coords.x<<"Y:"<<coordinates.tag_coords.y<<"Z:"<<coordinates.tag_coords.z<<std::endl;
                            }                            
                        }
                        else if(words[1]==NOT_UNDERSTAND+'\r')
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
            {
                words.push_back(word);
            }
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
    ElikoDriver():Node("eliko_driver"),anchor_modifier_(cloud_anchors_),tag_modifier_(cloud_tags_)
    {
        auto node=rclcpp::Node::make_shared("publisherData");
        rclcpp::Clock clock;
        this->declare_parameter("server_ip_",DEFAULT_SERVER_IP);
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
                std::cout<<"Connected to the server."<<std::endl;   
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
                std::cout<<"Anchors Obtained"<<std::endl;
                //Tag Cloud
                tag_modifier_.setPointCloud2Fields(3,
                    "x",1,sensor_msgs::msg::PointField::FLOAT32,
                    "y",1,sensor_msgs::msg::PointField::FLOAT32,
                    "z",1,sensor_msgs::msg::PointField::FLOAT32
                );
                tag_modifier_.reserve(SIZE);
                tag_modifier_.clear();
                setReportList(client_socket,node,clock);
                std::cout<<"program ended"<<std::endl;           
            }    
        }
    }
};