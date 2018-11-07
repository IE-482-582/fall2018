# Creating a Gazebo Model of the Duckiebot

Team Members:
- Chase Murray
- Somayeh Dejbord


## Project Objective
The goal of this project is to create a Gazebo model of the Duckiebot. This model will accurately reflect the dimensions of the Duckiebot, will include the Duckiebot's sensors (a fisheye lens camera and a magnetometer), and will have the same drive train (two motors controlling the two motorized wheels).


## Contributions
There are currently no Gazebo models of this robot.  By creating such a model, we will be able to test control algorithms in a simulated environment (without the need for the physical robot itself).  However, after training the control algorithms in Gazebo, it will be easy to execute them on a real Duckiebot, since the simulated version will be an accurate representation.


## Project Plan
The textbook contains two chapters (Chapters 15--17) that describe how to build a custom robot.
However, these chapters do not discuss the use of a fisheye lens.  We will use the ros.org Website to learn how to model such cameras.
We will also consult the Duckiebot specs to determine the dimensions and weight of the robot, as well as the capabilities of the motors.


## Milestones/Schedule
- Capture the specs of the actual/physical robot.  *SD, Nov. 16*
- Build a sample model using the textbook examples. *CM, Nov. 16*
- Modify the sample model to match the specs of the Duckiebot.  *CM, Nov. 19*
- Add a fisheye lens camera. *SD, Nov. 20*
- Create progress report.  *Due Nov. 30*
- Create Gazebo .launch files to test the robot.  *CM, Dec. 1*
- Create a simple controller to test the interaction with the robot. *SD, Dec. 3*
- Create final presentation.  *Due Dec. 5*
- Update documentation based on presentation feedback. *CM, Dec. 7*
- Provide system documentation (README.md).  *Due Dec. 14*




## Measures of Success
- View robot model in Gazebo.
- Demonstrate that the fisheye lens camera is appropriately distorted.
- Demonstrate that robot moves when given commands.
- Implement code on a real Duckiebot.
- Have a classmate follow the steps in the README to successfully run the simulation without any help.




