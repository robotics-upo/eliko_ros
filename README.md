Eliko_driver:
================
<table width="600">
    <tr>
        <td width="50%">
<img src="https://github.com/robotics-upo/eliko_ros/blob/pose_stamp_publisher/resources/Images/eliko_devices.jpg" alt="2d" width="400"/>
        </td>
        <td width="50%">
<img src="https://github.com/robotics-upo/eliko_ros/blob/pose_stamp_publisher/resources/Images/eliko_scheme.jpeg" alt="3d" width="400"/>
        </td>
    </tr>
</table>

Introduction
---
This is a ROS 2 driver for the Eliko RTLS created by the Service Robotics Lab from the Pablo de Olavide University (Spain).

This driver is used to connect to the Eliko RTLS system server and obtain all of the data related to the distances sent by it.

First, this driver is connected to the RTLS server via WiFi.\
After that, the driver sends a message to the server to accept the EULA to make the server start processing the next commands we will send to it.\
Once the EULA has been accepted, the driver sends two commands that will make the server start sending all the data related to the distances.\
After that, the driver receives all of the data and sends it to the user in custom messages.

This driver also creates multiple Points that shows the user the possition of each Tag in real time (The name of each tag is "T"+TagSerialNumber).

Before Downloading the code:
---
For this code to work properly, you must first do this steps.

- Have ros2, rviz 2, tf2 and colcon installed.
- Configure the network connection with the radar,

In this project, we are working with the **ROS 2 Humble** distribution on Ubuntu 22.04, so all of the ROS 2 commands that we will be using are from this particular distribution. In case you are using another ROS 2 distributionm this commands may change a bit.

* ### Install ROS 2:
    To install ROS2 you can follow this tutorial: <https://docs.ros.org/en/humble/Installation.html>

* ### Install RVIZ 2:
    This tool will be used to show the different objects that this driver will create to test it.\
    To install RVIZ 2, run this command on your terminal:
    ```
    sudo apt-get install ros-humble-rviz2 
    ```
* ### Install TF 2:
    To install TF 2 and all of its tools, run the next command on your terminal: 
    ```
    sudo apt-get install ros-humble-tf2-ros ros-humble
    ```
* ### Install Colcon:
    This tool is used to build ROS packages.\
    To install Colcon, execute the following command on your computer:
    ```
    sudo apt install python3-colcon-common-extensions
    ```

* ### Connect to the server and configure the Anchor possitions
    To do this configurations, you can follow the **Eliko RTLS Installation Guide** located in the documentation webpage:
    <https://eliko.tech/documentation/>
    
    (In this project we connect to our server using Wi Fi)

How to execute the driver
---
Once you have installed ROS 2, Rviz 2, Tf 2, colcon, connected to the server, configured the anchor positions and downloaded the project, you can execute this driver.

For executing the driver you should go to the directory in wich you have downloaded this project and execute the next commands:

* ```
    colcon build --packages-select eliko_driver eliko_messages
  ```
* ```
    source install/setup.bash
  ```
* ``` 
    ros2 launch eliko_driver eliko_launch.xml
  ```
The first command is used to build the project.\
The second command is used to source the project.\
The last command is used to launch the project and see the results in Rviz 2.

About the code:
---
This project is split into two packages: ELIKO_DRIVER and ELIKO_MESSAGES.

Now, we will start talking briefly about every file of this project focusing on what it does.
Firstly we will talk  about the files inside eliko_driver, and next, we will talk about the ones inside eliko_messages.
* ### eliko_driver.
    This package contains all of the code used to obtain the distances from the eliko server and send it to the user. (It also shows the coordinates of the tag in Rviz2)
    * ### eliko_driver.h
        This file contains all of the structures that are going to be used to store the data sent by the server and send it to the user.

    * ### eliko_driver.hpp
        This file contains the class that we will use to connect to our eliko server, obtain all fo the data related to the distances and publish it for later work.

        First of all, it creates the connection to the server and accepts the EULA.

        Then, it sends a message to make the server start sending us all of the data related to the distances.

        At last, it will start receiving all of the data requested and storing it in a specific structure wich will be sent to the user.(All of this happens in an infinite loop)
    
    * ### eliko_driver.cpp
        This file launches the driver class to execute the project and see how it works.
    
    * ### eliko_Visualization.rviz
        This file contains the configuration to show the anchors and tags in rviz 2.
    
    * ### eliko_launch.xml
        This is the file we will be executing using the following command:
        ```
        ros2 launch eliko_driver eliko_launch.xml
        ```
        Inside this file, we create and execute three nodes.
        * The first node is the one that is going to be executing the code created in this project.\
        This node has three parameters that we can change in case one of them does not correspond to the actual value.\
        These are the parameters:
            * The server IP.
            * The connection Port.
            * The Frame ID used to send and receive the different messages.
        
        * The second node will open an rviz 2 window to visualize the data related to the position received.\
        This node uses the **eliko_Visualization.rviz** file to configure the window.

        * The third and last node creates an static-transform-publisher to transform the data obtained from the server to the data shown in rviz 2 (The arguments inside this node can also be changed to adapt it to your project).

    * ### CMakeLists.txt and package.xml
        This two files configure and define all dependencies the project needs to execute the project. (Including libraries, packages, etc)

* ### eliko_messages
    This package contains all of the structures used to create the custom messages for the user.
    * ### Distances.msg
        This file has the data structure for the distance message. (We store the Serial number of the Tag, The Serial number of the Anchor and the distance between them)
    * ### DistancesList.msg
        This file has the structure for the distanceList message that is composed by an array of Distances messages.
    * ### AnchorCoords.msg
        This file has the data structure for the AnchorCoords message, that sends the position of the anchor and some extra information.(Including the id, role, serial number and the last time it stablished and lost the connection)
    * ### AnchorCoordsList.msg
        This file has the structure for the AnchorCoordList message that is composed by an array of AnchorCoord messages.
    * ### TagCoords.msg
        This file has the structure for the TagCoords message, that sends the Tag's Coordinates and some extra information. (Like it's serial number and some error information)
    * ### TagCoordsList.msg
        This file has the structure for the TagCoordsList message that is composed by an array of TagCoords messages.

