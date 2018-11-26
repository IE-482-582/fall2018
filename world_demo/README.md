# Creating Custom Worlds/Mazes

This document describes how to create a simple world with a single turtlebot.


1. Grab the current github repo
	```
	cd ~/Downloads
	rm -rf fall2018
	git clone https://github.com/IE-482-582/fall2018.git
	```	

2. Create a catkin package:
	```
	cd ~/catkin_ws/src
	catkin_create_pkg world_demo
	```
	
3. Create a scripts directory:
	```
	cd ~/catkin_ws/src/world_demo
	mkdir scripts
	```
	
4. Make sure you have everything setup properly:
	```
	cd ~/catkin_ws/src/world_demo
	tree
	```
	
	It should look like this:
	```
	.
	├── CMakeLists.txt
	├── package.xml
	└── scripts
	```
	
5. Copy files from maze_generator into your new scripts directory.  **RUN EACH COMMAND ONE-AT-A-TIME.**
	```
	cd ~/Downloads/fall2018/maze_generator
	cp create_world.py ~/catkin_ws/src/world_demo/scripts/
	cp parseCSVstring.py ~/catkin_ws/src/world_demo/scripts/
	cp sample_racetrack.xls ~/catkin_ws/src/world_demo/scripts/
	```
	
6. Copy files from redball into your new scripts directory.  **RUN EACH COMMAND ONE-AT-A-TIME.**
	```
	cd ~/Downloads/fall2018/09_redball_code/code/redball/scripts
	cp kobukiTEST.launch.xml ~/catkin_ws/src/world_demo/scripts/
	cp redball.launch ~/catkin_ws/src/world_demo/scripts/
	cp redball.world ~/catkin_ws/src/world_demo/scripts/
	```
	
7. Edit the sample_racetrack.xls file in Libre Office Calc

8. Save as `sample_racetrack.csv` (export as `.csv` file)
	- File --> Save As...
	- Choose file type = "Text CSC (.csv)"
	- Name the file "sample_racetrack.csv"
	- Save
	
9. Run the python script to generate a `.world` file:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	python create_world.py
	```	

10. Rename the `course_ubusername.world` file to be `world_demo.world`:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	mv course_ubusername.world world_demo.world
	```
	
11. Rename the `redball.launch` file
	```
	cd ~/catkin_ws/src/world_demo/scripts
	mv redball.launch world_demo.launch
	```
	
12. Open `world_demo.launch` in a text editor
	- Replace `find redball` with `find world_demo`.
	- Replace `redball.world` with `world_demo.world`.
	- Save the `.launch` file.

13. Make your `world_demo` package:
	```
	cd ~/catkin_ws
	catkin_make
	```
	
14. Test out your new world:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	roslaunch world_demo world_demo.launch
	```
