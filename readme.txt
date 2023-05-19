Caitlin Conn ENPM661 Project 3 Phase 2 Readme.txt 

This text provides the necessary instructions to run code associated with Project 3 Phase 2 Parts 1 & 2.

This following section provides the necessary instructions to run the astar_proj3_phase1_part1.py file to output the results for 
an A* Algorithm Approach for a robot forward search problem.

###############################################################################################################################

Project 3 Phase 1 Important Library/Dependency Notes:
Imported required libraries include the following and are listed in the beginning of the python script:
	from collections import OrderedDict
	from datetime import datetime
	import matplotlib.pyplot as plt
	import numpy as np 
	import imutils
	import time
	import cv2 
	import math
	from math import radians

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
- Case3_Inputs

Google Drive link to video submissions: https://drive.google.com/drive/folders/1bSPP8uZCjlRF8FajT2sEKiug_nPpemUQ?usp=share_link
- Anyone with video link should have access to the 3 video examples.

Github Project Link: https://github.com/caitlinpconn/ENPM661_Project3_Phase01.git
	
###############################################################################################################################

Project 3 Phase 1 Code Execution Instructions: 
To run the code to solve the search problem using Astar algorithm in Python, perform the following: 

If using a command line terminal, put the astar_caitlin_conn_submission.py file in the current working directory and enter the following 
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

Enter robot radius (ex. 5): 5
Robot Radius: 5

Enter robot clearance (ex. 5): 5
Robot Clearance: 5

Enter start node's x coordinate. x coordinate can range from 0 to 299: 10
Enter start node's y coordinate. y coordinate can range from 0 to 124: 11
Enter the orientation of the robot at the start point in degrees: 0
Start node x-coordinate: 10
Start node y-coordinate: 11

Enter goal node's x coordinate. x coordinate can range from 0 to 299: 225
Enter goal node's y coordinate. y coordinate can range from 0 to 124: 15
Enter the orientation of the robot at the goal point in degrees: 30
Goal node x-coordinate: 225
Goal node y-coordinate: 15

Enter step size of the robot in units (1 <= L <= 10): 10
Step Size L: 10

_______________________________________________________________________________________

Map Created and User Input Saved

Main code execution has started...

Last Child Node (Goal Node): 
 ((224.5, 16.0), (261.1180339887499, 3012, 2012, (216.0, 11.0), 260))

Problem solved, now backtrack to find optimal path!

_______________________________________________________________________________________

Was goal found ? ->  True

- Problem solved in (hours:min:sec:milliseconds) (Method 1): 00:00:00.69
- Problem solved in (hours:min:sec:milliseconds) (Method 2): 0:00:00.693808

Start node coordinate input: (10, 11)
Goal node coordinate input: (225, 15)

Code Script Complete.

###############################################################################################################################
