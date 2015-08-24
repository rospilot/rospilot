rospilot
========

1. Install the dependencies: `rosdep install rospilot`
2. Compile: `catkin_make`
3. Configure PostGIS, mapping server, and other services: `rosrun rospilot first_time_setup.sh`
4. Start rospilot: `roslaunch rospilot rospilot.launch`
5. The UI will now be running at http://localhost:8085

I am providing code in this repository to you under an open source license. Because this is my personal repository, the license you receive to my code is from me and not from my employer (Facebook).


Running linter
==============
`catkin_make roslint_rospilot`


Running tests
=============
`catkin_make test`
