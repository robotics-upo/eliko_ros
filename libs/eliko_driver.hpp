/**
 * @file eliko_driver.hpp
 * @brief This file include all the functions that obtain the data of the anchors, tags and live possitioning of the tags.
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
#include "Eliko_data.h"

#define DEFAULT_PORT 25025
#define DEFAULT_SERVER_IP "10.8.4.1"
#define POINTCLOUD_HEIGHT 1
#define SIZE 1000
#define DEFAULT_FRAME_ID "Eliko" 
#define rr_l "RR_L"
#define AnchorCoord "ANCHOR_COORD"
#define AnchorCoordE "ANCHOR_COORD_E"
#define coord "COORD"
#define coord_E "COORD_E"
#define getAnchorCommand "$PEKIO,GET_ANCHORS\r\n"
#define getAnchorCommandE "$PEKIO,GET_ANCHORS_E\r\n"
#define getRRlCommand "$PEKIO,SET_REPORT_LIST,RR_L\r\n"
#define getCoordCommand "$PEKIO,SET_REPORT_LIST,COORD\r\n"
#define getCoordECommand "$PEKIO,SET_REPORT_LIST,COORD_E\r\n"
#define getRRlCoordCommand "$PEKIO,SET_REPORT_LIST,RR_L,COORD\r\n"
#define getRRlCoordECommand "$PEKIO,SET_REPORT_LIST,RR_L,COORD_E\r\n"
#define getEverythingCommand "$PEKIO,SET_REPORT_LIST,RR_L,COORD,COORD_E\r\n"

class eliko_driver: public rclcpp::Node{
    private:
    sensor_msgs::msg::PointCloud2 cloudTags;
    sensor_msgs::msg::PointCloud2 cloudAnchors;
    sensor_msgs::PointCloud2Modifier anchorModifier;
    sensor_msgs::PointCloud2Modifier tagModifier;
    /**
     * @brief Puts the headers to all of the PointCloud messages that will be sent to rviz.
    */
    void fillCloudMessage(sensor_msgs::msg::PointCloud2 &cloud_msg){
        cloud_msg.header=std_msgs::msg::Header();
        cloud_msg.header.frame_id=this->frame_ID;
        std::chrono::time_point<std::chrono::system_clock> now=std::chrono::system_clock::now();
        auto duration=now.time_since_epoch();
        cloud_msg.header.stamp.nanosec=std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        cloud_msg.header.stamp.sec=std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        cloud_msg.is_dense=false;
        cloud_msg.is_bigendian=false;
        cloud_msg.height=POINTCLOUD_HEIGHT;
    }
    /**
     * @brief Sends the message to obtain the Anchor Coords to the server, obtains all the data sent back by the server and stores them in a struct.
    */
    void getAnchorCoords(int clientSocket,rclcpp::Node::SharedPtr node)
    {
        int bytesRead;
        char buffer[1024];
        auto publisherAnchor=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudAnchors",10);
        sensor_msgs::PointCloud2Iterator<float>pos_X(cloudAnchors,"x");
        sensor_msgs::PointCloud2Iterator<float>pos_Y(cloudAnchors,"y");
        sensor_msgs::PointCloud2Iterator<float>pos_Z(cloudAnchors,"z");
        send(clientSocket,getAnchorCommand,sizeof(getAnchorCommand),0);
        while((bytesRead=recv(clientSocket,buffer,sizeof(buffer),0))>0){
            fillCloudMessage(cloudAnchors);
            std::vector<std::string> lines;
            std::istringstream iss(buffer);
            std::string line;
            while (std::getline(iss,line))
            {
                lines.push_back(line);
            }
            for (auto& l:lines)
            {
                std::istringstream ss(l);
                std::vector<std::string>words;
                std::string word;
                while(std::getline(ss,word,','))
                {
                    words.push_back(word);
                }
                if(words[1]==AnchorCoord){    
                    ANCHOR_COORD anchor;
                    anchor.anchorID=words[2];
                    anchor.anchorRole=words[3][0];
                    anchor.anchorSN=words[4];
                    anchor.anchorPosition.X=stod(words[5]);
                    anchor.anchorPosition.Y=stod(words[6]);
                    anchor.anchorPosition.Z=stod(words[7]);
                    *pos_X=anchor.anchorPosition.X;
                    *pos_Y=anchor.anchorPosition.Y;
                    *pos_Z=anchor.anchorPosition.Z;
                    anchor.lastConnectionEstablished=words[8];
                    anchor.lastConnectionLost=words[9];
                    anchor.connectionState=stoi(words[10]);
                    ++pos_X;
                    ++pos_Y;
                    ++pos_Z;
                }
            }
            publisherAnchor->publish(cloudAnchors);  
        }   
    }
    void setReportList(int clientSocket,rclcpp::Node::SharedPtr node){
        char buffer[1024];
        int bytesRead;
        auto publisherAnchor=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudAnchors",10);
        send(clientSocket,getRRlCoordCommand,sizeof(getRRlCoordCommand),0);
        while((bytesRead=recv(clientSocket,buffer,sizeof(buffer),0))>0){
            std::vector<std::string> lines;
            std::istringstream iss(buffer);
            std::string line;
            while (std::getline(iss,line))
            {
                lines.push_back(line);
            }
            for (auto& l:lines)
            {
                std::istringstream ss(l);
                std::vector<std::string>words;
                std::string word;
                while(std::getline(ss,word,','))
                {
                    words.push_back(word);
                }
                if(words[1]==rr_l){
                    RR_L distances; 
                    int messageLength=words.size();
                    int numAnchorsRelatedMessages=messageLength-4;/*Not sure if I have to substract 4 to get rid of
                    the fields that are not Anchor related or a bigger number.*/
                    int numAnchorsDataMessages=numAnchorsRelatedMessages/3;//Because there are 3 different types of Anchor fields.
                    distances.seqNumber=stoi(words[2]);
                    distances.tagSN=words[3];
                    for (int i = 0; i < numAnchorsDataMessages; i++)
                    {
                        distances.anchors->anchorID=words[4+i*2];
                        distances.anchors->distance=stoi(words[5+i*2]);
                    }
                    distances.timestamp=words[5+numAnchorsDataMessages*2];
                    for(int i=0;i<numAnchorsDataMessages; i++)
                    {
                        distances.flags->flag=words[6+numAnchorsDataMessages*2+i];
                    }
                    distances.tagError.flag=words[messageLength-1];
                }else if(words[1]==coord){
                    COORD coordinates;
                    coordinates.seqNumber=stoi(words[2]);
                    coordinates.tagSN=words[3];
                    if(words[4]!=""){
                        coordinates.tagCoords.X=stod(words[4]);
                        coordinates.tagCoords.Y=stod(words[5]);
                        coordinates.tagCoords.Z=stod(words[6]);
                    }
                    coordinates.Info=words[7];
                    coordinates.timestamp=stod(words[8]);
                }
            }
        }
        for (int i =0;i<1025;i++){
            buffer[i] = '\0';
        }

    }
    
    public:
    std::string frame_ID;
    std::string serverIP;
    int serverPort;

    eliko_driver():Node("eliko_driver"),anchorModifier(cloudAnchors),tagModifier(cloudTags){
        auto node=rclcpp::Node::make_shared("publisherData");
        this->declare_parameter("serverIP",DEFAULT_SERVER_IP);
        this->declare_parameter("serverPort",DEFAULT_PORT);
        this->declare_parameter("frameID",DEFAULT_FRAME_ID);
        int clientSocket= socket(AF_INET,SOCK_STREAM,0);
        if(clientSocket==-1){
            std::cerr<<"Socket creation error"<<std::endl;
        
        }else{
            sockaddr_in serverAddress;
            serverAddress.sin_family=AF_INET;
            serverAddress.sin_port=htons(DEFAULT_PORT);
            serverAddress.sin_addr.s_addr=inet_addr(DEFAULT_SERVER_IP);
            if (connect(clientSocket,(struct sockaddr*)&serverAddress, sizeof(serverAddress))==-1){
                std::cerr<<"Problem trying to connect to the server"<<std::endl;
                close(clientSocket);
            }else{
                std::cout<<"Connected to the server."<<std::endl;   
                rclcpp::Clock clock;
                this->frame_ID=this->get_parameter("frameID").as_string();
                this->serverIP=this->get_parameter("serverIP").as_string();
                this->serverPort=this->get_parameter("serverPort").as_int();
                anchorModifier.setPointCloud2Fields(3,
                    "x",1,sensor_msgs::msg::PointField::FLOAT32,
                    "y",1,sensor_msgs::msg::PointField::FLOAT32,
                    "z",1,sensor_msgs::msg::PointField::FLOAT32
                );
                anchorModifier.reserve(SIZE);
                anchorModifier.clear();
                getAnchorCoords(clientSocket,node);
                tagModifier.setPointCloud2Fields(3,
                    "x",1,sensor_msgs::msg::PointField::FLOAT32,
                    "y",1,sensor_msgs::msg::PointField::FLOAT32,
                    "z",1,sensor_msgs::msg::PointField::FLOAT32
                );
                tagModifier.reserve(SIZE);
                tagModifier.clear();
                setReportList(clientSocket,node);
            }    
        }
    }

};