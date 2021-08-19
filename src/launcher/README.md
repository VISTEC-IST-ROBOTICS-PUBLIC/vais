# Launcher
This package acts as a bundle to combine every necessary parts from each package in order to perform a big main task at the same time. 

For example, The complete launch of ICO needs four nodes assembled together in order to use ICO as an experiment. The list of launcher is explained below:

ico_launch
- detector
- signal
- ico_launch
- output

The example of command to use a launcher is rosluanch launchers ico_launch.launch

##NOTE
https://answers.ros.org/question/226588/how-do-i-create-a-desktop-application-for-ros/
http://wiki.ros.org/roslaunch

#Logs
Aug 21 2019
- Update launcher file, CMakeList.txt, package.xml to force it using roslaunch
- Rename to launchers (to make a bit more different that general command, also fix the header on both CMakeList.txt, package.xml
