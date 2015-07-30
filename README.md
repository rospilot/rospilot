rospilot
========

1. Install the dependencies: `rosdep install rospilot`
2. Configure PostGIS, mapping server, and other services: `rosrun rospilot first_time_setup.sh`
3. Start rospilot: `roslaunch rospilot rospilot.launch`
4. The UI will now be running at http://localhost:8085

I am providing code in this repository to you under an open source license. Because this is my personal repository, the license you receive to my code is from me and not from my employer (Facebook).


Running linter
==============
`catkin_make roslint_rospilot`


Running tests
=============
`catkin_make test`


ARM Cross Compiler Installation
=======
* requirements:
  * arm-none-eabi-gcc cross compiler
    * `sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded`
    * `sudo apt-get update`
    * `sudo apt-get install gcc-arm-none-eabi`
