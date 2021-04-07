Install/Run Instructions for Ros2 with Webots project

If you would rather watch a video link that gives a visual walk through, follow this link to the “soft illusion” channel’s first tutorial video. Link: https://youtu.be/jU_FD1_zAqo
I highly recommend using both the video and the documentation.

Install ROS2:

1.) First I would recommend installing Ubuntu 20.04 for use of Ros2. There are instructions for installing Ros2 on Windows and on MacOS, but can be more difficult on both. For this documentation I will explain how to install on Ubuntu 20.04. 
-If you would like to dual boot your laptop so it can run Windows or Ubuntu upon startup, try this method, it has worked for me. Follow directions in the link very carefully
 Link: https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/


2.) Download the newest version of webots if you do not have it already. The link will bring you to a download screen, any type of download is fine as long as it is r2021a (currently newest version)
Link: https://cyberbotics.com/


3.) Install VS code to use as an editor for your projects. When using Ros2 with Webots, the coding and editing will take place outside of the Webots window. VS Code is a great and free editor for this. Use any installation type, debian package is recommended. Link: https://code.visualstudio.com/


4.) Now we want to download Ros2 Foxy. You have the option of installing or building Ros2 onto your laptop or PC. For what we are doing it is not necessary to build Ros2 but that is okay to do. I will link the installation directions for each platform. The Linux options are the easiest to do and it works best with Ubuntu 20.04. I have run into some errors with newer versions of Ubuntu and Mint when trying those installations. Those errors may have been fixed by now.
Linux Link: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html
Linux Debian Link: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
Windows Link: https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html
MacOS Link: https://docs.ros.org/en/foxy/Installation/macOS-Install-Binary.html
 

5.) We now want to clone the repository of the Webots/Ros2 github so that we can use the two together. There are many example programs that come with this repository that can be tried, or deleted.
Link: https://github.com/cyberbotics/webots_ros2
There are instructions on what commands to use to actually build and install this repository found at the bottom of the page. I have linked the installation and tutorials below as well for ease of access.
Link install: https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install
Tutorial Link: https://github.com/cyberbotics/webots_ros2/wiki/Tutorials

Install/Run Lab3:

My github repository: https://github.com/treevor34/ROS2_Lab3_Webots

1) Download Ros2 foxy for Linux preferably. Ensure that you download Colcon build and have that working with Ros2. I also downloaded Visual Studio Code and Webots, the newest versions of both.

2) Download the Webots_Ros2 workspace on github. There are install instructions that make this much easier. Linked: https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install

3) Within ros2_ws/src/webots_ros2 I created a new folder named “webots_lab3_task2”.

4) Inside the folder location for lab3, you create a folder for “launch”, “worlds”, and another folder of the same name as the main folder, in my case “webots_lab3_task2”. 

5) Place the world file in worlds, launch file in launch, and the two controller files in the third folder. The __init__.py should also be placed with the two controller files.

6) Now place “package.xml”, “setup.py”, and “setup.cfg” outside of those three folders just created.

7) Change directory to the beginning at ros2_ws. Type “source install/local_setup.bash”. Then run “colcon build --packages-select webots_lab3_task2”. Finally run “ros2 launch webots_lab3_task2 lab3_task2_launch.py”. This should open webots in a separate window if everything works properly.

8) While running, open a separate terminal and type “ros2 topic echo” and then tab twice. This will show the options to see in the terminal from the robot sensors. Simply type “ros2 topic echo” again followed by the choice you wish to see.

(Important): If your files are not connecting to the world correctly as a controller. Go to the world file and scroll until you see “DEF epuck E-puck”. Once there ensure that the controller area a few lines below looks like, “    controller "<extern>"    ”. This will allow you to link your files to the robot.

Creation/Understanding of Lab3:

The way Ros2 works is there should be 2 or more files used for having your robot do what you want. One file is devoted to getting readings from the robot itself. It will then “publish” the readings to the other file or files. It also published the change in the robot’s speed to the robot so that we can essentially control it. At least one other file is needed and it will essentially store all the logic necessary. This file “subscribes” to the robot and gets the readings that the other file publishes. After doing the logic this robot will send back specified velocities to the file that is communicating with the robot. This is essentially how we use the Ros2 controller with Webots. My communicator file is named “Slave” while my Logic file is named “Master”. This mirrors the names of the files from the youtube video mentioned at the top. 
If creating a new project, then I would recommend keeping the same structure of “def main(args=None):” found at the bottom of my two files because those are crucial to running the file and need not be changed unless changing the names. If looking at my master.py file I have a line that I changed to “fp = FindPoles()”. I changed the FindPoles() to match the name of the class in the same file. 
The contents of the communication or the publishing file is as follows. It gets the sensor devices and enables them. It then creates publishers for those sensors and publishes the values to the other file. The logic file creates subscriptions and gets data from the publishers via a callback. All of these values are being transmitted as a message which has different types. There is an exception to the way publishers and subscribers are set up. In the logic file instead of subscribing, we create a velocity publisher to publish the speeds we want to the communicator file. The communicator file subscribes and makes the changes by calling “motor.setVelocity(speed)”. This is the basics to the controller files.
For the launch, setup, and package files I largely just used the examples as templates for those. Inserting my world and controller files into the locations where the examples had their files at.
