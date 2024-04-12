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
     * @brief Sends the message to obtain the Anchor Coords to the server.
    */
    void getAnchorCoords(int clientSocket)
    {
        send(clientSocket,getAnchorCommand,sizeof(getAnchorCommand),0);
        
    }

    /**
     * @brief Reads all the messages sent by the server, filters them, creates new messages and sends the data to Rviz
    */
    int readData(int socket){
        auto node=rclcpp::Node::make_shared("publisherData");
        auto publisherAnchors=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudAnchors",10);
        auto publisherTags=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudTags",10);
        char buffer[1024];
        int bytesRead;

        while (true)
        {
        sensor_msgs::PointCloud2Iterator<float>iter_xA(cloudAnchors,"x");
        sensor_msgs::PointCloud2Iterator<float>iter_yA(cloudAnchors,"y");
        sensor_msgs::PointCloud2Iterator<float>iter_zA(cloudAnchors,"z");

        sensor_msgs::PointCloud2Iterator<float>iter_xT(cloudTags,"x");
        sensor_msgs::PointCloud2Iterator<float>iter_yT(cloudTags,"y");
        sensor_msgs::PointCloud2Iterator<float>iter_zT(cloudTags,"z");
       /*     std::cout<<"Do you want to get the Anchors coords?(Y, N)";
            char response;
            std::cin>>response;
            if(response=='Y'||response=='y'){ */
                getAnchorCoords(socket);
                while((bytesRead=recv(socket,buffer,sizeof(buffer),0))>0){
                    std::vector<std::string> lines;
                    std::istringstream iss(buffer);
                    std::string line;
                    while (std::getline(iss,line))
                    {
                        lines.push_back(line);
                    }
                    std::vector<std::vector<std::string>>wordsByLine;
                    for (auto& l:lines)
                    {
                        std::istringstream ss(l);
                        std::vector<std::string>words;
                        std::string word;
                        while(std::getline(ss,word,','))
                        {
                            words.push_back(word);
                        }
                        wordsByLine.push_back(words);
                    }
                    for(auto &words:wordsByLine){
                        std::string messageID=words[1];
                        if(messageID==AnchorCoord){
                            struct ANCHOR_COORD response;
                            fillCloudMessage(cloudAnchors);
                            response.anchorID=words[3];
                            response.anchorRole=words[4][0];
                            response.anchorSN=words[5];
                            response.anchorPosition.X=stod(words[6]);
                            response.anchorPosition.Y=stod(words[7]);
                            response.anchorPosition.Z=stod(words[8]);
                            response.lastConnectionEstablished=words[9];
                            response.lastConnectionLost=words[10];
                            response.connectionState=stoi(words[11]);
                            *iter_xA=response.anchorPosition.X;
                            *iter_yA=response.anchorPosition.Y;
                            *iter_zA=response.anchorPosition.Z;
                            ++iter_xA;
                            ++iter_yA;
                            ++iter_zA;
                            anchorModifier.resize(SIZE);
                            
                            publisherAnchors->publish(cloudAnchors);
                        }
                    }
                }
                for (int i =0;i<1025;i++){
                buffer[i] = '\0';
                }
           /* }else if(response!='N'&&response!='n'&&response!='Y'&&response!='y'){
                std::cout<<"Invalid answer";
                return -1;
            }*/
            send(socket,getRRlCoordCommand,sizeof(getRRlCoordCommand),0);
            while((bytesRead=recv(socket,buffer,sizeof(buffer),0))>0){
                std::vector<std::string> lines;
                std::istringstream iss(buffer);
                std::string line;
                while (std::getline(iss,line))
                {
                    lines.push_back(line);
                }
                std::vector<std::vector<std::string>>wordsByLine;
                for (auto& l:lines)
                {
                    std::istringstream ss(l);
                    std::vector<std::string>words;
                    std::string word;
                    while(std::getline(ss,word,','))
                    {
                        words.push_back(word);
                    }
                    wordsByLine.push_back(words);
                }
                for(auto &words:wordsByLine){
                    std::string messageID=words[1];
                    if (messageID==rr_l){//Check if it works.
                        std::cout<<"TagID: "<<words[3]<<"\nAnchor1ID: "<<words[4]<<"\nAnchor1Distance:"<<words[5]<<"\n";
                        struct RR_L response;
                        response.seqNumber=stoi(words[2]);
                        response.tagSN=words[3];
                        int numOfWords=words.size();
                        int anchorData=numOfWords-6;
                        int anchorCoords=anchorData/3;
                        for (int i = 0; i < anchorCoords; i++)
                        {
                            response.anchors[i].anchorID=words[4+(i*2)];
                            response.anchors[i].distance=stoi(words[5+(i*2)]);
                        }
                        response.timestamp=words[4+anchorCoords*2];
                        for (int i = 0; i < anchorCoords; i++)
                        {
                            response.flags[i].flag=words[5+(anchorCoords*2)+i];
                        }
                        response.tagError.flag=words[5+anchorData];
                    }else if (messageID==coord)
                    {
                    std::cout<<"TagID: "<<words[3]<<"\n X,Y,Z Coords: \nX: "<<words[4]<<"\nY: "<<words[5]<<"\nZ:"<<words[6]<<"\n";
                    struct COORD response;
                    fillCloudMessage(cloudTags);
                    response.seqNumber=stoi(words[2]);
                    response.tagSN=words[3];
                    if(words[4]!=""){
                        response.tagCoords.X=stoi(words[4]);
                        response.tagCoords.Y=stoi(words[5]);
                        response.tagCoords.Z=stoi(words[6]);   
                    }
                    response.Info=words[7];
                    response.timestamp=stod(words[8]);
                    *iter_xT=response.tagCoords.X;
                    *iter_yT=response.tagCoords.Y;
                    *iter_zT=response.tagCoords.Z;
                    ++iter_xT;
                    ++iter_yT;
                    ++iter_zT;
                    tagModifier.resize(SIZE);
                    publisherTags->publish(cloudTags);
                    }
                }
            }
            for (int i =0;i<1025;i++){
                buffer[i] = '\0';
            }

        }
        
    }    

    public:
    std::string frame_ID;
    std::string serverIP;
    int serverPort;

    eliko_driver():Node("eliko_driver"),anchorModifier(cloudAnchors),tagModifier(cloudTags){
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
            tagModifier.setPointCloud2Fields(3,
                "x",1,sensor_msgs::msg::PointField::FLOAT32,
                "y",1,sensor_msgs::msg::PointField::FLOAT32,
                "z",1,sensor_msgs::msg::PointField::FLOAT32
            );
            tagModifier.reserve(SIZE);
            tagModifier.clear();
            readData(clientSocket);
            }
        }
    }

};