# ROS2_Lab3_Webots
Install/Run Instructions:

My github repository: https://github.com/treevor34/ROS2_Lab3_Webots

1) Download Ros2 foxy for Linux preferably. Ensure that you download Colcon build and have that working with Ros2. I also downloaded Visual Studio Code and Webots, the newest versions of both.

2) Download the Webots_Ros2 workspace on github. There are install instructions that make this much easier. Linked: https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install

3) Within ros2_ws/src/webots_ros2 I created a new folder named “webots_lab3_task2”.

4) Inside the folder location for lab3, you create a folder for “launch”, “worlds”, and another folder of the same name as the main folder, in my case “webots_lab3_task2”. 

5) Place the world file in worlds, launch file in launch, and the two controller files in the third folder. The __init__.py should also be placed with the two controller files.

6) Now place “package.xml”, “setup.py”, and “setup.cfg” outside of those three folders just created.

7) Change directory to the beginning at ros2_ws. Type “source install/local_setup.bash”. Then run “colcon build --packages-select webots_lab3_task2”. Finally run “ros2 launch webots_lab3_task2 lab3_task2_launch.py”. This should open webots in a separate window if everything works properly.

8) While running, open a separate terminal and type “ros2 topic echo” and then tab twice. This will show the options to see in the terminal from the robot sensors. Simply type “ros2 topic echo” again followed by the choice you wish to see.

