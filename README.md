rospilot
========

1. Clone the rospilot_deps repo (https://github.com/rospilot/rospilot_deps)
2. Install the dependencies: `rosdep install -i rospilot && rosdep install rospilot_deps`
3. Configure PostGIS, mapping server, and other services: `rosrun rospilot first_time_setup.sh`

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
