# redball files

This package directs a turtlebot to follow a ball around a Gazebo world.

---

## Installation Instructions

1. Clone the course repository:
   ```
   cd ~/Downloads
   rm -rf fall2018
   git clone https://github.com/IE-482-582/fall2018.git
   ```
   
2. Run the `redball` installation script:
   ```
   cd ~/Downloads/fall2018/09_redball_code
   chmod +x install_redball.sh
   ./install_redball.sh
   ```

---

## Finding the Right Color Masks

Setting the appropriate HSV ranges can be tricky.  The `color_filter_test.py` script will help you to find the correct values.

You'll need two (2) terminal windows for this.

1. Terminal 1 -- Launch Gazebo

   ```
   cd ~/catkin_ws/src/redball/scripts
   roslaunch redball redball.launch
   ```

2. Terminal 2 -- Run the Color Filter Script

   ```
   cd ~/catkin_ws/src/redball/scripts
   rosrun redball color_filter_test.py
   ```

The last script will generate some images and `.csv` files in the scripts directory.  You may open the `.csv` files in Excel (or other spreadsheet app).  Each cell will contain `[h, s, v]` values.  It will be helpful to also have the images open so you can determine which cell corresponds with which part of the image.

---

## Make the Robot follow the Ball

You'll need three (3) terminal windows.

1. Terminal 1 -- Launch Gazebo

   ```
   cd ~/catkin_ws/src/redball/scripts
   roslaunch redball redball.launch
   ```

2. Terminal 2 -- Start the Robot Controller

   ```
   cd ~/catkin_ws/src/redball/scripts
   rosrun redball move_robot.py
   ```

3. Terminal 3 -- Move the Ball

   ```
   cd ~/catkin_ws/src/redball/scripts
   rosrun redball move_ball.py
   ```

---

## Editing/Customizing the Code

1. Take a look at the top of the `move_robot.py` script.  You'll see that there are some parameter values for you to change.  Experiment with different parameter values.  In particular, look at the options for `self.trackMethod`.

2. You may notice that the Hough Circles controller is very noisy.  See if you can improve the performance.

3. You should also look at the parameters in the `move_ball.py` script.
