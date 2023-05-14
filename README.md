# [Installing ROS and other tools](http://wiki.ros.org/noetic/Installation/Ubuntu)

1. Follow the install instruction on the [ROS website](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Add the following lines to your .bashrc file
    ```
    source /opt/ros/noetic/setup.bash
    alias s="$HOME/catkin_ws/src/devel/setup.bash"
    ```
3. Install python-is-python3
    ```
    sudo apt install python-is-python3
    ```
4. Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) to be able to compile code
    ```
    sudo apt install python3-catkin-tools
    ```

5. Install terminator terminal
    ```
    sudo apt install terminator
    ```

6. Install the teleop ROS package (to control the vehicles with the keyboard)
    ```
    sudo apt install ros-noetic-teleop-twist-keyboard
    ```
7. Install other dependencies that you might need in your project
    ```
    sudo apt install ros-noetic-gmapping ros-noetic-amcl ros-noetic-map-server
    ```

# [Creating your ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

1. Create a catkin workspace that will contain the code
    ```
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin build
    ```

2. Inside the src folder, create a new python-based ROS package
    ```
    cd src
    catkin_create_pkg demo_python std_msgs rospy
    ```

3. Open the package.xml file and edit it in order to look something like the one in this repository

4. As you need to add messages (or code) from other packages, you should include them in the package.xml file. For example, if you want to use the [geometry_msgs](http://wiki.ros.org/geometry_msgs) package, you should add the following line to the package.xml file
    ```
    <depend>geometry_msgs</depend>
    ```

5. Open the CMakeLists.txt file and edit it in order to look something like the one in this repository

6. As you need to add message (or code) from other packages, you should include them also in the CMakeLists.txt file. This may seem like duplicate work, but it is needed. For example, if you want to use the [geometry_msgs](http://wiki.ros.org/geometry_msgs) package, you should add the following line to the package.xml file
    ```
    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
    )

    catkin_package(
        CATKIN_DEPENDS 
        geometry_msgs 
    )
    ```

7. If you need to create custom message files, you should add them to the msg folder. For example, if you want to create a custom message called MyMessage, you should create a file called MyMessage.msg inside the msg folder. Then, you should add the following line to the CMakeLists.txt file
    ```
    add_message_files(
        FILES
        MyMessage.msg
    )
    ```

8. Create a Python ROS node program. For that, inside the src folder, create a new file called demo_node.py and add the code you want. For example, you can copy the code from the demo_node.py file in this repository.

9. In order for the python node to be executable by ROS, you need to make it executable
    ```
    chmod +x demo_node.py
    ```
    
10. To compile and index the package by the ROS system, run:
    ```
    cd ~/catkin_ws/
    catkin build
    ```
11. Add the following lines to your bashrc file, in order for the system to know that your code exists and can be executed:
    ```
    nano ~/.bashrc
    
    # Add this line to the .bashrc file and save
    source $HOME/catkin_ws/devel/setup.bash
    ```
12. Source the modified .bashrc file:
    ```
    source ~/.bashrc
    ```
13. Try to run the node by running:
    ```
    rosmaster
    rosrun demo_python demo_python.py
    ```

14. Alternatively, use the launch file (and with it, you can avoid having to launch a roscore manually):
    ```
    roslaunch demo_python example.launch
    ```
