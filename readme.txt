Caitlin Conn ENPM661 Project 3 Phase 2 Readme.txt 

This text provides the necessary instructions to run code associated with Project 3 Phase 2 Parts 1 & 2.

###############################################################################################################################

This following section provides the necessary instructions to run the Project 3 Phase 2 Part 1 astar_proj3_phase1_part1.py file to output the results for 
an A* Algorithm Approach for a robot forward search problem.

###############################################################################################################################

Project 3 Phase 1 Part 1 Important Library/Dependency Notes:
Imported required libraries include the following and are listed in the beginning of the python script:
	from collections import OrderedDict
	from datetime import datetime

	import matplotlib.pyplot as plt
	import numpy as np 
	import imutils
	import math
	import time
	import cv2 

Additional packages are required to be installed using the following commands depending on the user's version of Python:
- pip3 install opencv-python
- pip install images2gif
- pip install pillow
- pip install vidmaker
- pip install pygame
- pip install imutils

*Please note that due to how many code versions the author made, not all library or package depedencies may be captured 
in this Readme file. Upon code excution, the user's terminal will warn the user if they are missing any required dependencies, 
which will result in an error when running the .py file. The dependencies must be installed for the code to run.

###############################################################################################################################

Description of User Test Case Inputs for the Submitted Video Files are located in the Test Cases folder in the submission folder 
with the following file names:
- Case1_Inputs
- Case2_Inputs

Google Drive link to project 3 phase 2 part 1 video submissions: https://drive.google.com/drive/folders/1hzMyxIBugktPPcxX4xuWx1_8e_hoqY7F?usp=share_link
- Anyone with video link should have access to the 2 video examples.

Github Project Link: https://github.com/caitlinpconn/ENPM661_Project3_Phase02.git
	
###############################################################################################################################

Project 3 Phase 1 Part 1 Code Execution Instructions: 
To run the code to solve the search problem using Astar algorithm in Python, perform the following: 

If using a command line terminal, put the astar_proj3_phase1_part1.py file in the current working directory and enter the following 
command in the terminal to run the python program using Python 3 version:
 
	python3 ./astar_proj3_phase1_part1
	
Otherwise, use an IDE to open and run the astar_proj3_phase1_part1.py file, by hitting the 'run' button.

-------------------------------------------------------------------------------------------------------------------------------

The user should enter the code input parameters by specifying the inputs when prompted by the code using an IDE or command line terminal.

An example of the code at the start of execution and prompt for user input is shown below. 
The grid origin (0,0) is defined in the bottom left corner of the map.

For example, user enters using keyboard 10 followed by enter and then 11 followed by enter to specify the inital node input.
User then enters using keyboard 15 followed by enter and then 20 followed by enter to specify the goal node input.

The code will output if a solution exists for the inputed problem and if the goal node was found.
The code will output a video animation called 'conn_astar_algorithm_video.avi'if a solution was found to the current directory and the code will output the time the algorithm to run takes in hours, minutes, seconds, and milliseconds using two different methods.

An example of user input in a Python terminal and program execution is provided below.

###############################################################################################################################

Enter robot clearance between 0 and 15: 10
Robot Clearance: 10

Enter robot Revolutions per Minute (RPM1) integer value between 10 and 55: 10
Robot RPM1: 10

Enter robot Revolutions per Minute (RPM2) integer value between 10 and 55: 50
Robot RPM2: 50

Enter start node's x coordinate. x coordinate can range from 0 to 599: 50
Enter start node's y coordinate. y coordinate can range from 0 to 199: 100
Enter the orientation of the robot at the start point in degrees: 30
Start node x-coordinate: 50
Start node y-coordinate: 100

Enter goal node's x coordinate. x coordinate can range from 0 to 599: 500
Enter goal node's y coordinate. y coordinate can range from 0 to 199: 25
Goal node x-coordinate: 500
Goal node y-coordinate: 25

_______________________________________________________________________________________

Map Created and User Input Saved

Main code execution has started...

Debug Counter:  5000
Current Parent Node:
((226.57147542009707, 30.093792115701653), (824.2760263932516, 6747, 6180, (227.51104888293966, 25.03529150711062), 85.71574550852918, 277.324091495639))

Debug Counter:  10000
Current Parent Node:
((227.13602833151492, 60.08934703166599), (843.0967824406827, 10450, 10044, (228.72142469456193, 55.19468207160679), 122.85957584754863, 292.87497513090864))

Debug Counter:  15000
Current Parent Node:
((203.1758227960719, 43.89677980552384), (849.7118715766466, 13650, 12648, (205.08817294563033, 40.07953427418677), 135.2408526272218, 254.86170402247203))

Debug Counter:  20000
Current Parent Node:
((185.83199080192625, 56.581066191896504), (844.8953389318461, 25422, 25421, (181.6332919864291, 49.03058199947674), 60.9531919491829, 213.39268099508692))

Debug Counter:  25000
Current Parent Node:
((444.1106791097799, 10.546249719073508), (713.3011703711428, 31774, 31769, (438.9716268414251, 10.298523090888878), 17.618723220326892, 597.8450819781373))

Last Child Node (Goal Node): 
 ((497.2159004435019, 28.362180797035045), (662.7315649763623, 31820, 31815, (493.51785709410376, 26.22842309593554), 654.0010506610555))

Problem solved, now backtrack to find optimal path!

_______________________________________________________________________________________

Was goal found ? ->  True

- Problem solved in (hours:min:sec:milliseconds) (Method 1): 00:00:13.50
- Problem solved in (hours:min:sec:milliseconds) (Method 2): 0:00:13.499867

Start node coordinate input: (50, 100)
Goal node coordinate input: (500, 25)

Astar and Visual Code Complete.

###############################################################################################################################

This following section provides the necessary instructions to run the Project 3 Phase 2 Part 2 astar.py file to output the results for 
an A* Algorithm Approach for a robot forward search problem and simulating TurtleBot3 Burger in Gazebo.

###############################################################################################################################

Project 3 Phase 1 Part 2 Important Library/Dependency Notes:
Imported required libraries include the following and are listed in the beginning of the python script:
	from geometry_msgs.msg import Twist
	from nav_msgs.msg import Odometry

	from collections import OrderedDict
	from datetime import datetime

	import matplotlib.pyplot as plt
	import numpy as np 
	import imutils
	import rospy
	import math
	import time
	import cv2 

	from tf.transformations import euler_from_quaternion, quaternion_from_euler

Additional packages are required to be installed using the following commands depending on the user's version of Python:
- pip3 install opencv-python
- pip install images2gif
- pip install pillow
- pip install vidmaker
- pip install pygame
- pip install imutils

*Please note that due to how many code versions the author made, not all library or package depedencies may be captured 
in this Readme file. Upon code excution, the user's terminal will warn the user if they are missing any required dependencies, 
which will result in an error when running the .py file. The dependencies must be installed for the code to run.

###############################################################################################################################

Description of User Test Case Inputs for the Submitted Video Files are located in the Test Cases folder in the submission folder 
with the following file name:
- Case 1 & 2 Inputs

Google Drive link to project 3 phase 2 part 1 video submissions: https://drive.google.com/drive/folders/1sVXbZTO0UR_9T0ss7uUbVTjenOI3lXPH?usp=share_link
- Anyone with video link should have access to the 2 video examples.

Github Project Link: https://github.com/caitlinpconn/ENPM661_Project3_Phase02.git
	
###############################################################################################################################

Project 3 Phase 1 Part 2 Code Execution Instructions: 

To run the code to solve the search problem using Astar algorithm in Python, perform the following: 

Using a command line terminal, copy the provided catkin_ws in your current working directory.

In one terminal, enter the follow commands to launch Gazebo:
	cd ./catkin_ws && catkin_make
	source devel/setup.bash
	roslaunch proj3_gazebo world.launch
	
-------------------------------------------------------------------------------------------------------------------------------

In another terminal, enter the follow commands to run the astar.py file from the proj3_gazebo package:
	cd ./catkin_ws && catkin_make
	source devel/setup.bash
	rosrun proj3_gazebo astar.py

In the same terminal, the user should then enter the code input parameters by specifying the inputs when prompted by the code using the command line terminal.

An example of the code at the start of execution and prompt for user input is shown below. 

The code will output if a solution exists for the inputed problem and if the goal node was found.
The code will output a video animation called 'conn_astar_algorithm_video.avi'if a solution was found to the current directory and the code will output the time the algorithm to run takes in hours, minutes, seconds, and milliseconds using two different methods.

An example of user input in a Python terminal and program execution is provided below. 

* The user should close any matlibplot pop up windows before proceding to each step.

###############################################################################################################################
Enter robot clearance between 0 and 15: 15
Robot Clearance: 15

Enter robot Revolutions per Minute (RPM1) integer value between 10 and 55: 10
Robot RPM1: 10

Enter robot Revolutions per Minute (RPM2) integer value between 10 and 55: 50
Robot RPM2: 50

Enter start node's x coordinate. x coordinate can range from 0 to 599: 50
Enter start node's y coordinate. y coordinate can range from 0 to 199: 100
Enter the orientation of the robot at the start point in degrees: 0
Start node x-coordinate: 50
Start node y-coordinate: 100

Enter goal node's x coordinate. x coordinate can range from 0 to 599: 100
Enter goal node's y coordinate. y coordinate can range from 0 to 199: 25
Goal node x-coordinate: 100
Goal node y-coordinate: 25

###############################################################################################################################
