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
#define ANCHORCOORD "ANCHOR_COORD"
#define ANCHORCOORDE "ANCHOR_COORD_E"
#define COORD "COORD"
#define COORD_E "COORD_E"
#define NOT_UNDERSTAND "CANNOT_UNDERSTAND"
/**
 * These next define rows store the messages the user (driver) sends to the server to obtain
 * some data and some of the responses the server sends.
*/
#define GETEUKASTATUSCOMMAND "$PEKIO,EULA,STATUS\r\n"
#define ACCEPTEULACOMMAND "$PEKIO,EULA,ACCEPT\r\n"
#define GETANCHORCOMMAND "$PEKIO,GET_ANCHORS\r\n"
#define GETANCHORCOMMANDE "$PEKIO,GET_ANCHORS_E\r\n"
#define GETRRLCOMMAND "$PEKIO,SET_REPORT_LIST,RR_L\r\n"
#define GETCOORDCOMMAND "$PEKIO,SET_REPORT_LIST,COORD\r\n"
#define GETCOORDECOMMAND "$PEKIO,SET_REPORT_LIST,COORD_E\r\n"
#define GETRRLCOORDCOMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD\r\n"
#define GETRRLCOORDECOMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD_E\r\n"
#define GETEVERYTHINGCOMMAND "$PEKIO,SET_REPORT_LIST,RR_L,COORD,COORD_E\r\n"

class Eliko_driver: public rclcpp::Node{
    private:
    sensor_msgs::msg::PointCloud2 cloudTags;
    sensor_msgs::msg::PointCloud2 cloudAnchors;
    eliko_messages::msg::Distances DistanceToTags;
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
        send(clientSocket,GETANCHORCOMMAND,sizeof(GETANCHORCOMMAND),0);
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
                        if(words[1]==ANCHORCOORD){
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
        auto publisherDistance=node->create_publisher<eliko_messages::msg::Distances>("Distances",10);
        auto publisherTag=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudTags",10);
        send(clientSocket,GETRRLCOORDCOMMAND,sizeof(GETRRLCOORDCOMMAND),0);
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
                        if(words[1]==RR_L){
                            /**
                             * Here we get all the data received from the server under the identifier "RR_L" which
                             * sends us the distances between the Tag and all the anchors. 
                            */
                            rr_l distances; 
                            /**
                             * This three next lines are used because the number of fields of this message is variable.
                             * This variation depends on the number of anchors that participate in this particular message.
                            */
                            int messageLength=words.size()-1;
                            int numAnchorsRelatedMessages=messageLength-6;// Here we remove all of the fields that are not related to the anchors.
                            int numAnchors=(numAnchorsRelatedMessages/3);//Here we obtain the number of anchors that are detecting the Tag.
                            // We divide the total Anchor fields between 3 because there are 3 fields per Anchor
                            distances.seqNumber=stoi(words[2]);
                            distances.tagSN=words[3];
                            for (int i = 0; i < numAnchors; i++)
                            {
                                distances.anchors[i].anchorID=words[4+(i*2)];
                                distances.anchors[i].distance=stoi(words[5+(i*2)]);
                                DistanceToTags.header.frame_id=this->frame_ID;
                                DistanceToTags.header.stamp=clock.now();
                                DistanceToTags.anchor_sn=distances.anchors[i].anchorID;
                                DistanceToTags.distance=distances.anchors[i].distance;
                                DistanceToTags.tag_sn=distances.tagSN;
                                publisherDistance->publish(DistanceToTags);
                            }
                            distances.timestamp=words[6+(numAnchors*2)];
                            for(int i=0;i<numAnchors; i++)
                            {
                                distances.flags[i].flag=words[7+(numAnchors*2)+i];
                            }
                            std::istringstream ss2(words[messageLength-1]);
                            while (std::getline(ss2,word,'\r'))
                                /** 
                                 * We do this because every message ends with "\r\n" and when we split all of the messages,
                                 * we get rid of '\n' but not '\r' which appears at the end of the last field.
                                */  
                            {
                                lastValues.push_back(word);
                            }
                            
                            distances.tagError.flag=lastValues[0];
                            std::cout<<distances.tagSN<<std::endl;
                        }else if(words[1]==COORD){
                            /**
                             * Here, we obtain the data received from the server where the identifier is equal to "COORD".
                             * This message sends the possition calculated for a tag. It is asynchronous and is sent everytime a new position has been calculated.
                            */
                            coord coordinates;
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
                        }else if(words[1]==NOT_UNDERSTAND+'\r'){
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
     * @param clientSocket The socket where the client is having the conversation with the server.
    */
    void EULAStatus(int clientSocket){
        int bytesRead;
        char buffer[1024];
        send(clientSocket,GETEUKASTATUSCOMMAND,sizeof(GETEUKASTATUSCOMMAND),0);
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
                send(clientSocket,ACCEPTEULACOMMAND,sizeof(ACCEPTEULACOMMAND),0);
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
    Eliko_driver():Node("eliko_driver"),anchorModifier(cloudAnchors),tagModifier(cloudTags){
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