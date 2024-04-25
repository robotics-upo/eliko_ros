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
#define getEulaStatusCommand "$PEKIO,EULA,STATUS\r\n"
#define acceptEulaCommand "$PEKIO,EULA,ACCEPT\r\n"
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
     * @param cloud_msg The pointcloud message that needs to be filled.
    */
    void fillCloudMessage(sensor_msgs::msg::PointCloud2 &cloud_msg,rclcpp::Clock clock){
        cloud_msg.header=std_msgs::msg::Header();
        cloud_msg.header.frame_id=this->frame_ID;
        cloud_msg.header.stamp=clock.now();
        cloud_msg.is_dense=false;
        cloud_msg.is_bigendian=false;
        cloud_msg.height=POINTCLOUD_HEIGHT;
    }
    /**
     * @brief Sends the message to obtain the Anchor Coords to the server, obtains all the data sent back by the server and stores them in a struct.
     * @param clientSocket The Socket where the client is connected.
     * @param node  The Node created by the user.
    */
    void getAnchorCoords(int clientSocket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock)
    {
        int bytesRead;
        char buffer[1024]={0};
        auto publisherAnchor=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudAnchors",10);
        send(clientSocket,getAnchorCommand,sizeof(getAnchorCommand),0);
        while((bytesRead=recv(clientSocket,buffer,sizeof(buffer),0))>0){
            std::vector<std::string> lines;
            std::istringstream iss(buffer);
            std::string line;            
            while (std::getline(iss,line))
            {
                lines.push_back(line);
            }
            if(lines.size()>0){
                for (auto& l:lines)
                {
                    sensor_msgs::PointCloud2Iterator<float>pos_X(cloudAnchors,"x");
                    sensor_msgs::PointCloud2Iterator<float>pos_Y(cloudAnchors,"y");
                    sensor_msgs::PointCloud2Iterator<float>pos_Z(cloudAnchors,"z");
                    anchorModifier.resize(lines.size());
                    std::istringstream ss(l);
                    std::vector<std::string>words;
                    std::string word;
                    std::vector<std::string>lastValues;
                    std::string conState;
                    while(std::getline(ss,word,','))
                    {
                        words.push_back(word);
                    }
                    if(words.size()>1){
                        if(words[1]==AnchorCoord){
                            fillCloudMessage(cloudAnchors,clock);
                            //To eliminate the '/r' from the end of the line    
                            std::istringstream ss2(words[10]);
                            ANCHOR_COORD anchor;
                            anchor.anchorID=words[2];
                            anchor.anchorRole=words[3][0];
                            anchor.anchorSN=words[4];
                            
                            if(words[5]!=""){
                                anchor.anchorPosition.X=stof(words[5]);
                                anchor.anchorPosition.Y=stof(words[6]);
                                anchor.anchorPosition.Z=stof(words[7]);
                            }else{
                                anchor.anchorPosition.X=0;
                                anchor.anchorPosition.Y=0;
                                anchor.anchorPosition.Z=0;
                            }
                        
                            anchor.lastConnectionEstablished=words[8];
                            anchor.lastConnectionLost=words[9];
                            while(std::getline(ss2,word,'\r')){
                                lastValues.push_back(word);
                            }
                            conState=lastValues[0];
                            anchor.connectionState=stoi(conState);
                            //To fill up the pointCloud
                            *pos_X=anchor.anchorPosition.X;
                            *pos_Y=anchor.anchorPosition.Y;
                            *pos_Z=anchor.anchorPosition.Z;
                            publisherAnchor->publish(cloudAnchors);
                        }else if (words[1]=="EOF\r"){
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
     * @param clientSocket The socket where the conversation is maintained.
     * @param node The driver node to create the different publishers. 
    */
    void setReportList(int clientSocket,rclcpp::Node::SharedPtr node,rclcpp::Clock clock){
        char buffer[1024]={0};
        int bytesRead;
       
        auto publisherTag=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudTags",10);
        send(clientSocket,getRRlCoordCommand,sizeof(getRRlCoordCommand),0);
        while((bytesRead=recv(clientSocket,buffer,sizeof(buffer),0))>0){
            fillCloudMessage(cloudTags,clock);
            std::vector<std::string> lines;
            std::istringstream iss(buffer);
            std::string line;
            while (std::getline(iss,line))
            {
                lines.push_back(line);
            }
            if(lines.size()>0){
                tagModifier.resize(lines.size());
                for (auto& l:lines)
                {
                    sensor_msgs::PointCloud2Iterator<float>pos_XT(cloudTags,"x");
                    sensor_msgs::PointCloud2Iterator<float>pos_YT(cloudTags,"y");
                    sensor_msgs::PointCloud2Iterator<float>pos_ZT(cloudTags,"z");
                    std::istringstream ss(l);
                    std::vector<std::string>words;
                    std::string word;
                    std::vector<std::string> lastValues;
                    while(std::getline(ss,word,','))
                    {
                        words.push_back(word);
                    }
                    if(words.size()>1){
                        if(words[1]==rr_l){
                            RR_L distances; 
                            int messageLength=words.size()-1;
                            int numAnchorsRelatedMessages=messageLength-6;//Not sure if I have to substract 5 to get rid of
                            //the fields that are not Anchor related or a bigger number.
                            int numAnchorsDataMessages=(numAnchorsRelatedMessages/3);//Because there are 3 different types of Anchor fields.
                            distances.seqNumber=stoi(words[2]);
                            distances.tagSN=words[3];
                            for (int i = 0; i < numAnchorsDataMessages; i++)
                            {
                                distances.anchors[i].anchorID=words[4+(i*2)];
                                distances.anchors[i].distance=stoi(words[5+(i*2)]);
                            }
                            distances.timestamp=words[6+(numAnchorsDataMessages*2)];
                            for(int i=0;i<numAnchorsDataMessages; i++)
                            {
                                distances.flags[i].flag=words[7+(numAnchorsDataMessages*2)+i];
                            }
                            std::istringstream ss2(words[messageLength-1]);
                            while (std::getline(ss2,word,'\r'))
                            {
                                lastValues.push_back(word);
                            }
                            
                            distances.tagError.flag=lastValues[0];
                            std::cout<<distances.tagSN<<std::endl;
                        }else if(words[1]==coord){
                            COORD coordinates;
                            coordinates.seqNumber=stoi(words[2]);
                            coordinates.tagSN=words[3];
                            if(words[4]!=""){
                                coordinates.tagCoords.X=stof(words[4]);
                                coordinates.tagCoords.Y=stof(words[5]);
                                coordinates.tagCoords.Z=stof(words[6]);
                                *pos_XT=coordinates.tagCoords.X;
                                *pos_YT=coordinates.tagCoords.Y;
                                *pos_ZT=coordinates.tagCoords.Z;
                                std::vector<std::string>lastValues;
                                std::string timestamp;
                                std::istringstream ss2(words[words.size()-1]);
                                coordinates.Info=words[7];
                                while(std::getline(ss2,word,'\r')){
                                    lastValues.push_back(word);
                                }
                                timestamp=lastValues[0];
                                coordinates.timestamp=stoi(timestamp);
                                publisherTag->publish(cloudTags);
                                std::cout<<"X:"<<coordinates.tagCoords.X<<"Y:"<<coordinates.tagCoords.Y<<"Z:"<<coordinates.tagCoords.Z<<std::endl;
                            }                            
                        }else if(words[1]=="EOF\r"){
                            std::cout<<"returning"<<std::endl;
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
     * @param clientSocket The socket where the client is having the conversation with the server.
    */
    void EULAStatus(int clientSocket){
        int bytesRead;
        char buffer[1024];
        send(clientSocket,getEulaStatusCommand,sizeof(getEulaStatusCommand),0);
        while((bytesRead=recv(clientSocket,buffer,sizeof(buffer),0))>0){
            std::istringstream iss(buffer);
            std::vector<std::string>words;
            std::string word;
            while (std::getline(iss,word,','))
            {
                words.push_back(word);
            }
            if(words[1]=="NOT_GOOD"){
                std::cout<<"There has been a problem"<<std::endl;
            }else if(words[2]=="NOT_ACCEPTED"){
                send(clientSocket,acceptEulaCommand,sizeof(acceptEulaCommand),0);
                EULAStatus(clientSocket);
            }else{
                std::cout<<"EULA Accepted"<<std::endl;
                return;
            }
        }
    }
    
    public:
    std::string frame_ID;
    std::string serverIP;
    int serverPort;
    /**
     * @brief eliko_driver Node. Used to try the driver.
    */
    eliko_driver():Node("eliko_driver"),anchorModifier(cloudAnchors),tagModifier(cloudTags){
        auto node=rclcpp::Node::make_shared("publisherData");
        rclcpp::Clock clock;
        this->declare_parameter("serverIP",DEFAULT_SERVER_IP);
        this->declare_parameter("serverPort",DEFAULT_PORT);
        this->declare_parameter("frameID",DEFAULT_FRAME_ID);
        this->frame_ID=this->get_parameter("frameID").as_string();
        this->serverIP=this->get_parameter("serverIP").as_string();
        this->serverPort=this->get_parameter("serverPort").as_int();
        int clientSocket= socket(AF_INET,SOCK_STREAM,0);
        if(clientSocket==-1){
            std::cerr<<"Socket creation error"<<std::endl;
        }else{
            RCLCPP_DEBUG(node->get_logger(),"Socket Creado");
            sockaddr_in serverAddress;
            serverAddress.sin_family=AF_INET;
            serverAddress.sin_port=htons(DEFAULT_PORT);
            serverAddress.sin_addr.s_addr=inet_addr(DEFAULT_SERVER_IP);
            if (connect(clientSocket,(struct sockaddr*)&serverAddress, sizeof(serverAddress))==-1){
                std::cerr<<"Problem trying to connect to the server"<<std::endl;
                std::cout<<"Error connecting to the server"<<std::endl;
                RCLCPP_DEBUG(node->get_logger(),"Problema conectandome");
                close(clientSocket);
            }else{
                RCLCPP_DEBUG(node->get_logger(),"Conectado");
                std::cout<<"Connected to the server."<<std::endl;   
                rclcpp::Clock clock;
                //To accept the EULA and receive all of the messages from eliko.
                EULAStatus(clientSocket);  
                //Set fields and size of every PointCloud
                //AnchorCloud
                anchorModifier.setPointCloud2Fields(3,
                    "x",1,sensor_msgs::msg::PointField::FLOAT32,
                    "y",1,sensor_msgs::msg::PointField::FLOAT32,
                    "z",1,sensor_msgs::msg::PointField::FLOAT32
                );
                anchorModifier.reserve(SIZE);
                anchorModifier.clear();
                getAnchorCoords(clientSocket,node,clock);
                std::cout<<"Anchors Obtained"<<std::endl;
                //Tag Cloud
                tagModifier.setPointCloud2Fields(3,
                    "x",1,sensor_msgs::msg::PointField::FLOAT32,
                    "y",1,sensor_msgs::msg::PointField::FLOAT32,
                    "z",1,sensor_msgs::msg::PointField::FLOAT32
                );
                tagModifier.reserve(SIZE);
                tagModifier.clear();
                setReportList(clientSocket,node,clock);
                std::cout<<"program ended"<<std::endl;
                
            }    
        }
    }

};