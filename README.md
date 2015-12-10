KTH DD2425 2015 Group 7
===================

Contributors
------------
Jesper Karlsson, Ondrej Holesovsky, Guisong Fu, Aimen Chikhaoui

Introduction
------------
The purpose of this project is to create a small robot which is able to maneuver a maze of varied size and complexity. The maze simulates a search and rescue situation, where the task is to locate and fetch small objects, as well as avoid potential debris.

How to Compile
--------------
Run 
	```
	catkin_make 
	```
in the workspace of interest

How to Run
----------
There are three launch files located in the launch package needed in order to run the system.
In all cases the camera launch file needs to be run, and then choose between 2.1 and 2.2 depending on what phase you want to run. 
	
1. Launch Camera	
	(start the camera and check if data is published)
	```
	roslaunch launch camera.launch 
	```
2. 
	1. Launch Exploration
		```
		roslaunch launch explore.launch
		```
   	2. Launch Fetching 
   		``` 
		roslaunch launch fetch.launch
		```

Record Data
-----------
If you want to record data from the system you can use.
	```
	roslaunch launch record_small.launch
	```
