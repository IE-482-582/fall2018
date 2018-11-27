# Creating Custom Worlds/Mazes for Gazebo

There are 3 sections in this document.  
- The first section describes how to create and setup the `world_demo` catkin package.
- The second section describes how to create a world/maze with numerous unit cubes and a single turtlebot.
- The last section describes how to build a parking lot world.

---

## 1) Setup the `world_demo` Package.

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
	
5. Copy files from `maze_generator` into your new scripts directory.  **RUN EACH COMMAND ONE-AT-A-TIME.**
	```
	cd ~/Downloads/fall2018/maze_generator
	cp create_world.py ~/catkin_ws/src/world_demo/scripts/
	cp parseCSVstring.py ~/catkin_ws/src/world_demo/scripts/
	cp sample_racetrack.xls ~/catkin_ws/src/world_demo/scripts/
	```
	
6. Copy files from `redball` into your new scripts directory.  **RUN EACH COMMAND ONE-AT-A-TIME.**
	```
	cd ~/Downloads/fall2018/09_redball_code/code/redball/scripts
	cp kobukiTEST.launch.xml ~/catkin_ws/src/world_demo/scripts/
	cp redball.launch ~/catkin_ws/src/world_demo/scripts/
	```

7. Make your `world_demo` package:
	```
	cd ~/catkin_ws
	catkin_make
	```

---

## 2) Creating a Custom Maze World

![maze image][screenshots/maze.png]

1. Edit the `sample_racetrack.xls` file in Libre Office Calc.

2. Save as `sample_racetrack.csv` (export as `.csv` file):
	- File --> Save As...
	- Choose file type = "Text CSC (.csv)"
	- Name the file "sample_racetrack.csv"
	- Save
	
3. Run the python script to generate a `.world` file:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	python create_world.py
	```	

4. Rename the `course_ubusername.world` file to be `world_demo.world`:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	mv course_ubusername.world world_demo.world
	```
	
5. Rename the `redball.launch` file
	```
	cd ~/catkin_ws/src/world_demo/scripts
	mv redball.launch world_demo.launch
	```
	
6. Open `world_demo.launch` in a text editor
	- Replace `find redball` with `find world_demo`.
	- Replace `redball.world` with `world_demo.world`.
	- Save the `.launch` file.

	
7. Test out your new world:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	roslaunch world_demo world_demo.launch
	```

8. Want to start your turtlebot in a different location?
	- Open `kobukiTEST.launch.xml`
	- Edit the line that looks like this:
		```
		args="$(optenv ROBOT_INITIAL_POSE) -unpause -urdf -param robot_description -model mobile_base -x 1.5 -z 10"/>
		```
		You can add a `-y 0.0` parameter to set the y coordinate.
		
---		

## 3) Creating a Parking Lot World

![parking lot image][screenshots/parking_lot.png]

1. Copy files from `~/Downloads/world_demo` into your catkin workspace directory.  **RUN EACH COMMAND ONE-AT-A-TIME.**
	```
	cd ~/Downloads/fall2018/world_demo
	cp create_parking_world.py ~/catkin_ws/src/world_demo/scripts/
	cp parking_lot_test.world ~/catkin_ws/src/world_demo/scripts/
	cp parking_lot.launch ~/catkin_ws/src/world_demo/scripts/
	cp sample_parking_lot.xls ~/catkin_ws/src/world_demo/scripts/
	cp sample_parking_lot.csv ~/catkin_ws/src/world_demo/scripts/
	```

2. Edit the `sample_parking_lot.xls` file in Libre Office Calc.
	NOTES:
	- This spreadsheet has 4 columns that you need to fill in.
		- `x` and `y` are the coordinates of the **midpoint** of the object.
		- `type` may only be the following: 
			- `lane` -- a white line in the parking lot.
			- `car` -- a vehicle to occupy a parking space.
			- `curb` -- a raised object.
		- `length` describes the longest dimension of the object.  For `car`, `length` should equal 0 (it's ignored).
		- `angle` is the rotation of the object.  A horizontal `lane` or `curb` will be at 0 degrees.  However, the pickup model appears to be rotatated, so a horizontal pickup should be at 90 degrees.
	- The graph in the Excel spreadsheet isn't terribly helpful, as it doesn't show the line lengths.
	
3. Save as `sample_parking_lot.csv` (export as `.csv` file):
	- File --> Save As...
	- Choose file type = "Text CSC (.csv)"
	- Name the file "sample_parking_lot.csv"
	- Save
	
3. Run the python script to generate a `.world` file:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	python create_parking_world.py
	```	

4. If you'd like, you may rename the `parking_lot_test.world` file...**just be sure the file ends in `.world`**.  For example, here's how to rename the file to be `parking_lot_2.world`:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	mv parking_lot_test.world parking_lot_2.world
	```
		
6. Open `parking_lot.launch` in a text editor
	- Replace `parking_lot_test.world` with the name of the file from the previous step (e.g., `parking_lot_2.world`).
	- Save the `.launch` file.

	
7. Test out your new world:
	```
	cd ~/catkin_ws/src/world_demo/scripts
	roslaunch world_demo parking_lot.launch
	```

	NOTE: 
	- The first time you run the launcher, ROS will need to download the models for the asphalt background and the pickup.  Be patient.
	
8. If you use the pickup model, you might notice that it's rather big.  Here's how you can reduce the scale:
	```
	cd ~/.gazebo/models/pickup/meshes
	pico pickup.dae
	```
	
	Scroll down to about line 11, where you'll see `<unit name="inch" meter="0.0254"/>`.  I found that making one inch equal 0.0154 meters does a pretty good job.

	
	
	
