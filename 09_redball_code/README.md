# redball files

## To Do

<STRIKE>
1. Create a new package named `redball`
   - Don't forget to edit the `CMakeLists.txt` and `package.xml` files.  See the `followbot` package for examples.
   
2. Edit `redball.launch`, `redball.world`, and `redball.material` files to reference the new `redball` package.
   - Make sure there are **no** references to `followbot`.
</STRIKE>

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

3. Test that you're able to launch Gazebo:

   ```
   cd ~/catkin_ws/src/redball/scripts
   roslaunch redball redball.launch
   ```
   
4. Move the ball so it's in view of the camera
   - You'll need to edit `redball.world`.
   
5. Fix the lighting.  Notice how dark the underside of the ball is?
   - Again, take a look at `redball.world`.

6. Test the following command, which will move the red ball (make sure you're running Gazebo first):

   ```
   rosservice call /gazebo/set_model_state '{model_state: { model_name: red_ball_1, pose: { position: { x: 2, y: 0, z: 0.5 }, orientation: {x: 0, y: 0 , z: 0, w: 0} }, twist: { linear: {x: 0, y: 0, z: 0}, angular: { x: 0, y: 0, z: 0}}, reference_frame: world}}'
   ```
   
7.  Write a ROS node that issues this command.  Save this node as `move_ball.py`.

8. Edit `move_ball.py` so it "continuously" moves the ball around (within the boundaries of the world, of course).
   - You might want the ball to stop periodically.
   - You are encouraged to make the ball move in x-, y-, and z-space.
   - Make sure the ball doesn't move too fast, though.
      
9. Write a ROS node that will control your robot.  Save this node as `move_robot.py`.
   - I highly recommend that you use your old `followbot` code as a starting point.
   
10. In your `move_robot.py` script: 
	1. Create a mask that will detect the red ball;
    2. Determine the size of the ball;
    3. Draw a box around the ball; 
    4. Determine the robot's distance from the ball; and
    5. Make your robot follow the ball from a consistent distance.

11. Create a `README.md` file for your new package.
