rospilot
========

1. Install the udev rules in etc/: sudo cp etc/rospilot.rules /etc/udev/rules.d/
2. Install the dependencies in apt-get-deps.txt: sudo apt-get install `cat apt-get-deps.txt`

I am providing code in this repository to you under an open source license. Because this is my personal repository, the license you receive to my code is from me and not from my employer (Facebook).

PX4
========
* Steps
  1. clone https://github.com/cvg/px-ros-pkg into src/
  2. git reset --hard c3d6e12ad3aa187d993eb9584a76fbd8a5621feb

ARM Cross Compiler Installation
=======
* requirements:
  * arm-none-eabi-gcc cross compiler
    * sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
    * sudo apt-get update
    * sudo apt-get install gcc-arm-none-eabi
