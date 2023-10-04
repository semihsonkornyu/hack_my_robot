# Hack My Robot
###### System Instructions 

### Robot Preparation
1. Build your robot.
   * [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly)
   * [Youtube tutorial](https://www.youtube.com/watch?v=5D9S_tcenL4) 
2. Setup your PC.
   * [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)
   * Download and install Ubuntu on your PC (Remote PC). The recommended distribution is Noetic. You're free to install everything on a virtual machine, but performance wise it's going to be better if you partition your disk. 
   * Install dependent ROS packages and TurtleBot3 packages.
   * Clone this repository on your ```catkin_ws``` workspace and build it. (You can also clone and run this node on the TurtleBot3, but performance wise we recommend you run it on your Remote PC)
   ```
   cd catkin_ws
   catkin_make
   ```
3. Setup TurtleBot SBC.
   * Either follow [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup) to flash their ROS image and set up the TurtleBot.
   * Or flash the micro SD card with the provided [.img file](https://drive.google.com/file/d/18IKNXmm-hT8XVjJUEMNhZLNY1hveEXTp/view?usp=sharing) from the HMR organisers. This image is built based on the one provided by Robotis in their tutorial, and you can achieve the same results by following the full tutorial.
   * Configure the WiFi Network Settings
      -  Open a terminal window and go to the ```/etc/netplan``` directory in the microSD card.
      - Edit the 50-cloud-init.yaml file to modify your WIFI_SSID and your WIFI_PASSWORD. 
        ```
        sudo nano 50-cloud-init.yaml
        ```
        ![](doc/images/2022-10-17-10-35-43.png)
   * Plug the microSD card back into the turtlebot and boot it up. If all the steps were done correctly, the turtlebot should now be connected to your WiFi network and you can connect through ssh (username:ubuntu / password:turtlebot).
        ```
        ssh ubuntu@IP_ADDRESS_OF_TURTLEBOT3
        ``` 
4. ROS Network configuration.
   * Adjust the parameters according to the IP addresses of your devices.
   * Remote PC.
      - ROS_MASTER_URI=http://localhost:11311
      - ROS_IP={IP_ADDRESS_OF_REMOTE_PC}
      - Include the IP address and hostname of the TurtleBot3 in your ```/etc/hosts``` file (if you don't do this, you might have issues communicating with the TurtleBot3 later on).
  
   * TurtleBot3 SBC. 
      - ROS_MASTER_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
      - ROS_IP={IP_ADDRESS_OF_TURTLEBOT3}
      - Include the IP address and hostname of your Remote PC in the TurtleBot3 ```/etc/hosts``` file (if you don't do this, you might have issues communicating with the TurtleBot3 later on).
  
5. Test your configuration.
   * Bringup your robot by following the [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup).
   * Test the [keyboard teleoperation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/) from your Remote PC to check if all the network configuration parameters have been properly set. If the robot is not moving, you probably need to recheck all the network parameters, double check that the /etc/hosts file in the TurtleBot3 contains a reference to your Remote PC.  

### Mapping the scenario
For further detailed information please refer to the [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node).

1. ```roscore``` on Remote PC.
2. Bringup of the robot on the TurtleBot3 if not running already. 
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3. Use the SLAM algorithm to map the scenario. It doesn't need to be too big, just focus on the area you are going to work with, around 1.5mx1.5m. 
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
4. Use the keyboard teleoperation to drive your robot around, until you are satisfied with the mapped environment.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
5. Save your map. You can use the *-f* option in order to specify a folder location and a file name. 
```
rosrun map_server map_saver -f ~/map
```
6. Kill the SLAM node with Ctrl+C

### Launch the Hack My Robot routine

The launch file located in ```hack_my_robot/launch/hack_my_robot_complete.launch``` already launches the navigation node, the image_saver node and the hack_my_robot main routine node. Refer to the [Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes) for further information regarding the navigation stack. 

1. Edit the launch file to suit your parameters.
   * Map name and location for the navigation node.
   * Any other parameter you might have modified (although we don't recommend that).  
2. Launch the file. 
```
roslaunch hack_my_robot hack_my_robot_complete.launch
```
3. Setup your initial position.
![](doc/images/2022-10-17-10-37-55.png)
4. Launch the keyboard teleoperation node in order to move your robot around so it can locate iself on the map (just spinning around should be enough).
5. Once the robot has located itself, we need to change the functionality of the 2D Pose Estimate button. We will use the same tool in order to provide our robot the 3 waypoints needed for the Hack My Robot main routine. 
   * Go to the Panels dropdown menu and make the "Tools Properties" panel active.
   * Change the topic of the 2D Pose Estimate tool from */initialpose* to */addpose*
    ![](doc/images/2022-10-17-11-04-31.png) 
   * Close the panel. 
6. Indicate 3 positions in the map using the 2D Pose Estimate tool, in the same way as you indicated the initial position of the robot in Step 3. A small green arrow should appear where you indicated the position, and a message should appear on the terminal where you launched the ```hack_my_robot_complete.launch file```. 
7. Publish an empty message to the path_ready topic to run the routine once and save the positions in a file. 
```
rostopic pub -1 /path_ready std_msgs/Empty
```
8. Wait until it finishes the routine. If all went well, the robot should have "EXCAVATED" in the 3 locations you previously set. 
9. If you are happy with the routine, publish an empty message to the start_journey topic to start a loop. The routine will stop as soon as you stop publishing the message to the topic. 
```
rostopic pub /start_journey std_msgs/Empty
```
10.  Let's get hacking!!

###### Final Round Instructions 

### Evaluation Criteria

The criteria are aligned with the first-round questions, and the teams are expected to implement some of the ideas they provided in response to the related questions. Moreover, the given criteria aim to evaluate the teams’ ability to reach the goals of the competition while being creative and original. The teams will be ranked based on the total points collected from all the items listed below.

The evaluation criteria and necessary explanations are as follows:

**1. Did the team need the Wi-Fi password to proceed with the competition?**

The Wi-Fi password used by the robot will be provided to any team that requests it. However, the teams that did not ask for the Wi-Fi password at any point during the final round will receive additional points. As an alternative, if the team was given a list of passwords that includes the actual password will receive some points as well. Different possible points are as follows: 

  1.1) The team asked for the Wi-Fi password **(0 Points)**
  
  1.2) The team asked for a list of possible Wi-Fi passwords **(5 Points)** 
  
  1.3) The team did not ask for the Wi-Fi password **(10 Points)**

**2. Is the team able to acquire/access the data collected by the robot and stored in the temporary or final storage?**

This criterion includes only accessing the data collected by the robot. The teams are expected to implement some of the ideas they provided in response to Question 1 in the first round. (In the context of this competition, the collected data corresponds to the images taken at each predefined location.)

  2.1) When the data is in temporary storage (low security) (Yes/No) **(5 points/0 points)**

    The robot will be temporarily storing the collected images during each round until they are transferred to the final storage (with a higher security level) at the end of the round. The team will collect 5 points if they are able to acquire the data while it is in temporary storage. 

  2.2) When the data is in the final storage (high security) (Yes/No) **(10 points/0 points)**

    The collected images will be transferred to the final storage (with a higher security level) at the end of each round. The team will collect 10 points if they are able to acquire the data while it is in the final storage.
    
**3. Is the team able to alter/modify the data collected by the robot?**

The team will collect points if they can change the data collected by the robot before it is transmitted to the high-security final storage in the external computer. (In the context of this competition, the collected data corresponds to the images taken at each predefined location.) The teams are expected to implement some of the ideas they provided in response to Question 1 in the first round. Points for different types of alterations are as follows:

3.1) Deleting the data (Yes/No) **(2 Points/0 Points)**

3.2) Corrupting the data (Yes/No) **(3 Points/0 Points)**

3.3) Replacing the original image with another image (Yes/No) **(5 Points/0 Points)**

3.4) Inserting malware hidden in an image (Yes/No) **(10 Points/0 Points)**

**4. Is the team able to alter the predefined path or the functionality (i.e., preventing it from taking images) the robot should follow? (Yes/No) (15 points/0 points)**

The team will collect points if they can achieve either one of the following:

  4.1) Make the robot follow a path different than the predefined one. (Making the robot collect data on the altered path is not required.)
  
  4.2.) Disrupt the robot’s functionality. 

**5. Is the team able to compromise the availability of the robot?**

The team will collect points if they can reduce the robot’s performance, cause interruptions in resource availability, or cause a total loss of availability. Please refer to the “2.3.3. Availability (A)” section of the [Common Vulnerability Scoring System (CVSS) v3.1 specification document](https://www.first.org/cvss/v3-1/cvss-v31-specification_r1.pdf) to see different levels of impact on availability. Different possible points based on the performance of the team are as follows:

  5.1) No impact on availability **(0 Points)**

  5.2) Low impact on availability **(7 Points)**
    
    Performance is reduced, or there are interruptions in resource availability. 

  5.3) High impact on availability **(15 Points)**
    
    Total loss of availability, which results in full denial of access to resources.  

**6. Technical difficulty/sophistication of the utilized techniques (0-15 points)**

The judges will evaluate the teams based on the complexity and sophistication of the methods they used during the final round. Please refer to the “2.1.2. Attack Complexity (AC)” section of the [Common Vulnerability Scoring System (CVSS) v3.1 specification document](https://www.first.org/cvss/v3-1/cvss-v31-specification_r1.pdf) to see what is expected in terms of complexity.

**7. Creativity (wow factor) (0-10 points)**

This criterion aims to evaluate the team’s ability to think “out of the box”. The teams will collect points for this criterion if they can develop original ideas to tackle the challenges mentioned in the previous criteria.

**8. Other/bonus points (up to 15 points)**

**Note:** The teams will have physical access to the robots; however, if they achieve the goals by utilizing physical attacks, they will receive half of the standard points. See the “2.1.1. Attack Vector (AV)” and "7.4. Metric Values" sections of the [Common Vulnerability Scoring System (CVSS) v3.1 specification document](https://www.first.org/cvss/v3-1/cvss-v31-specification_r1.pdf) for the justification. 

### Final Round Rules and Timeline

1. All teams should bring their laptops to the final round to perform their attacks. 
2. The teams will have physical access to the robots; however, if they achieve the goals by utilizing physical attacks, they will receive half of the standard points.
3. Each team will need to present a poster at the end of the final round. The template for the poster can be found [here](https://docs.google.com/presentation/d/1x-OmkGRheAjjI__kbQStaainIRyfAbNc/edit?usp=share_link&ouid=110937514079708301128&rtpof=true&sd=true). Each team will have 10 minutes to present/summarize what they did during the challenge (see the timeline in the item below). The poster presentation will be considered by the judges, too (e.g., in the Creativity / Bonus sections). The poster size is A0 (841 x 1189 mm). It can be printed at NYUAD if needed.
4. The final round will take place from 1 pm to 5 pm on November 11. The breakdown of this period is as follows:
    1. 2:00 - 2:30 pm (30 mins) - Preparation / set up
    2. 2:30 - 5:00 pm (2.5 hours) - Competition and judging
    3. 5:00 - 6:00 pm (1 hour) - Poster presentations (10 mins per team) / judges' deliberations
