# Weed-Sparying-Robot
This is the Final Project of SE423: Mechatronics.

The goal of this project is to program the robot to navigate to five different (x, y) points in their numbered order. Along the way to these five points locate and imaginatively “spray with herbicide” at least two of the five brightly colored weeds dispersed in the course and avoid the obstacles in the robot’s path. Once all five points have been reached, if needed continue locating and spraying weeds and then report the number of orange weeds found at the orange reporting circle and the number of blue weeds found at the blue reporting circle. The robot will be scored by the amount of time it takes to navigate the course and report at least two weeds in their respective circle. 

We have LIDAR, radar, camera sensors on the robot and use A* for path planning.

<p align="center">
  <img src=/robot.jpg>
</p>

Source code for obstacle detection/map building/weed detection/PID controller/state machine is in:
\Contest\FinalContestRobot2\source\user_FinalContestRobot2.c

All the code are compiled using Code Composer Studio v8.

Website that details the project: sites.goole.com/illinois.edu/bobthebot/home

Team members: Lance Zhou, Priyam Shah, Ashwin Azhagiri, Mateus Jonas, Gayatri Dandu
