# assisted_cleaning_solution (ACS)

The robot software for the assisted cleaning solution robot. The robot can navigation from room to room, clean plinths, find deskchairs, go to deskchair and clean the deskchair.

## Dependencies

noetic:

	map_server
	move_base
	teb_local_planner
	amcl
	rosbridge_server

python3:

	pip3

pip3:	

	scipy
	numpy
	pandas

## Needed packages

### ira_laser_tools-master

This package merges the lidars sensor data.
To use this package you need to install the ira_laser_tools-master package from source. 
The file laserscan_multi_merger.launch should look like:

	<launch>
		<arg name="node_start_delay" default="5.0" />  
		<node pkg="ira_laser_tools" name="laserscan_multi_merger" type="laserscan_multi_merger" output="screen" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' ">
				<param name="destination_frame" value="base_footprint"/>
				<param name="cloud_destination_topic" value="/merged_cloud"/>
				<param name="scan_destination_topic" value="/scan"/>
				<param name="laserscan_topics" value ="/scan1 /scan2" /> <!-- LIST OF THE LASER SCAN TOPICS TO SUBSCRIBE -->
				<param name="angle_min" value="-3.14"/>
				<param name="angle_max" value="3.14"/>
				<param name="angle_increment" value="0.0064"/>
				<param name="scan_time" value="0.0333333"/>
				<param name="range_min" value="0.1"/>
				<param name="range_max" value="9.0"/>
		</node>
	</launch>


### office_test_environment

This is a RoS noetic package for a office test environment. 
To use this package you need to install the office_test_environment package from source.
More information about the package in the git repo : https://github.com/fontysrobotics/office_test_environment.git
 
This package contains:
* Configuration data of joints.
* 3D model URDF.
* Simple test launch file.

### robot_assisted_cleaning_solution

This is a RoS noetic package for the Assisted Cleaning Solution robot.
More information about the package in the git repo : https://github.com/fontysrobotics/robot_assisted_cleaning_solution.git

This package contains: 
* Configuration data of joints.
* 3D model URDF.
* launch files. For testing, Mapping and Navigation.
* Configurations for the global planner: move_base.
* Configurations for the local planner: teb_local_planner.

### gui_assisted_cleaning_solution

Repository for the web user interface of the Assisted Cleaning Solution robot.
More information about the package in the git repo : https://github.com/fontysrobotics/gui_assisted_cleaning_solution.git

This package contains: 
* The HTML files of the webpages
* The CSS files of the webpages
* The Javascript files of the webpages
* The language files
* The images used in the page

## To start the simulation, robot software and web user interface

    roslaunch assisted_cleaning_solution nodes.launch
    python3 -m http.server 7000;
    rosrun assisted_cleaning_solution smach_assisted_cleaning_solution.py

## Package information

### Task message
The task message is used for giving the robot a task to perform. The user interface publishes the message by pressing the buttons for the tasks. The navigation tasks are the numbers from one to higher. The rooms can easily be extended in the positive range of the numbers. The tasks go from -2 to lower. These can also be easily extended.

#### Task message layout

	-3 = Chairs
	-2 = Plinths
	-1 = Error (does nothing yet)
    0 = Stop and Waiting for Task
    1 = Charging Station
    2 = Default Position in Room
    3 = Room 1
    4 = Room 2
    5 = Room 3
    6 = Room 4

#### Expand rooms.

1. Go to https://github.com/fontysrobotics/gui_assisted_cleaning_solution.git
2. Follow the steps under change or add number of rooms
3. Follow the steps under change floorplan
4. Go to the assisted_cleaning_solution package
5. Go to the csv folder and open RoomCoords
6. Add the new row with the next row number
7. In the same row add ;
8. In the same row add the coordinates of the new room with layout: 
	position x, position y, quaternion z, quaternion w
(These coordinates should be next to a wall with the right side of the AGV for starting with cleaning the plinths)
9. Repeat step 6 to 8 for all the rooms that needed to be added

#### Expand Tasks.

1. Go to https://github.com/fontysrobotics/gui_assisted_cleaning_solution.git
2. Follow the steps under add new task
3. Go to the assisted_cleaning_solution package
4. Go to the folder scripts the file util.py class receive_task
5. Add in the init a outcome of the new task.
6. Add in the execute a new statement with the new task. The statement should correspond with the value of the task message and the return value should be the outcome that was added in the init.
7. Go to the folder scripts the file smach_assisted_cleaning_solutions.py state Recieve_task add an additional transition for the new task.
8. Make a new state machine for the new task.

### Change simulation

#### Change environment

1. Go to the launch folder file nodes.launch
2. Change the first include to the new environment. 

For less chance of issues the new environment should have the same package layout as https://github.com/fontysrobotics/office_test_environment.git

#### Change robot

1. Go to the launch folder file nodes.launch
2. Change the second and third include to the new robot package

For less chance of issues with the new robot the package should have the same package layout as https://github.com/fontysrobotics/robot_assisted_cleaning_solution.git
With the same navigation package used.