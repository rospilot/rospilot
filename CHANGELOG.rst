^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospilot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2021-02-21)
------------------
* Fix systemd service to use noetic
* Remove map server
  Tilestache is no longer supported on modern Ubuntu
* Remove firmware directory
* Switch to Github Actions for CI
* Remove calls to deprecated ffmpeg functions
* Contributors: Christopher Berner

1.6.0 (2020-09-06)
------------------
* Add support for Noetic
* Update to Python3
* Upgrade jquery to fix CVE
* Contributors: Christopher Berner

1.5.6 (2019-03-30)
------------------
* Avoid thread exhaustion in h264 server
  Previously, connections didn't timeout, so a badly behaved client
  could exhaust all the threads on the host
* Contributors: Christopher Berner

1.5.5 (2019-03-29)
------------------
* Fix web_ui.py serving of nodejs dependencies
* Upgrade Bootstrap to fix CVEs
* Contributors: Christopher Berner

1.5.4 (2019-02-17)
------------------
* Update vendored NPM deps
* Remove hard npm dependency
  npm deps must now be vendored
* Remove Highcharts and accelerometer graph page
* Added missing wget dependency to package.xml that was found in first_time_setup.sh.
* Improve cold start H264 streaming to new clients
* Refactor H264 streaming to allow Android app support
* Contributors: Anders Fischer, Christopher Berner

1.5.3 (2018-10-08)
------------------
* Fix dnsmasq dependency on wlan device
* Contributors: Christopher Berner

1.5.2 (2018-09-22)
------------------
* Fix compiler warnings
* Fix SPS & PPS ordering when using MFC encoder
  SPS should always be before PPS, as PPS references the SPS packet
* Remove unnecessary C++11 compiler flag
* Fix undefined behavior in AVPacket buffer management
  Use the provided av_new_packet() and av_packet_unref()
  functions instead of manual management of .data field
* Fix undefined behavior in memory deallocation
  This buffer is an array, and therefore must be deallocated with delete[]
* Contributors: Christopher Berner

1.5.1 (2018-09-03)
------------------
* Fix wlan configuration on Bionic
* Fix Systemd service to work on Melodic
* Contributors: Christopher Berner

1.5.0 (2018-08-04)
------------------
* Upgrade to Angular 5.2.11
* Upgrade Highcharts
* Set both pts and dts in video recorder
* Properly set time_base on video recorder codec
* Remove usage of deprecated encoding/decoding functions
* Remove usage of deprecated AVPicture
* Fix AVStream::codec deprecation warnings
* Add error handling for call to avformat_write_header()
* Cleanup VideoRecorder::stop()
* Switch Travis to build melodic
* Update ffmpeg usage to compile against newest version
* Update environment variable to PGPASSWORD
* Switch to Melodic-friendly dependencies
* Hack: monkey patch Mapnik to fix Tilestache dependency
* Contributors: Christopher Berner

1.4.1 (2018-03-10)
------------------
* Switch to HTTPS for openstreetmap.org
* Workaround camera drivers that don't set keyframe flag
* Add support for reading raw YUV from camera
* Refactor pixel format handling
* Contributors: Christopher Berner

1.4.0 (2017-11-19)
------------------
* Add "follow me" mode
* Add support for direct h264 capture from camera
* Improve automatic camera selection
* Add vibration data to display
  Also includes accelerometer clipping counts
* Add support for battery voltage monitoring
* Enable pre-release tests
* Add Travis integration
* Refactor handling of NPM dependencies
* Add native Systemd service
* Set dns search domain for better compatibility
* Update wifi setup script with proper dnsmasq dependency
* Contributors: Christopher Berner

1.3.8 (2017-09-13)
------------------
* Remove dependency on nodejs-legacy
  Also remove usage of vendored npm version
* Contributors: Christopher Berner

1.3.7 (2017-06-10)
------------------
* Update for newest NPM
* Contributors: Christopher Berner

1.3.6 (2017-05-29)
------------------
* Add support for Pixhawk2.1
* Contributors: Christopher Berner

1.3.5 (2017-03-12)
------------------
* Add missing include needed on Debian
* Contributors: Christopher Berner

1.3.4 (2017-03-08)
------------------
* Rescale PTS properly
* Fix usage of deprecated API
* Fix usage of invalid iterator
* Add more logging to camera node
* Fix data structure corruption due to race
* Contributors: Christopher Berner

1.3.3 (2016-08-24)
------------------
* Fix build on Debian
* Contributors: Christopher Berner

1.3.2 (2016-08-24)
------------------
* Support building on Debian
* Contributors: Christopher Berner

1.3.1 (2016-08-21)
------------------
* Fix build on Wily
* Contributors: Christopher Berner

1.3.0 (2016-08-20)
------------------
* Initialize source frame data structure
* Update init script to Kinetic
* Set CherryPy to production instead of manually configuring it
* Use npm for build
* Use more canonical license string
* Fix scoping in set waypoint callback
* Improve mavlink wait logic to be interruptable
* Fix retry logic for serial mavlink
* Migrate to Angular 2.0
* Add timeout to waypoint fetching
  Previously, if a message was lost waypoints would no longer be fetched
  and new ones could not be set
* Contributors: Christopher Berner

1.2.0 (2016-05-14)
------------------
* Add people detector using OpenCV
* Contributors: Christopher Berner

1.1.1 (2016-01-31)
------------------
* Copy files to /etc as part of setup script instead of package install
* Fix warnings and installation of mapnik files
* Add missing python-serial dependency
* Cleanup linking of libturbojpeg
  This should fix compiling on other platforms like x86_32
* Contributors: Christopher Berner

1.1.0 (2016-01-18)
------------------
* Auto-detect camera device path
* Improve video streaming FPS ~2x
* Add support for hardware h264 acceleration on Odroid XU4
* Add FPS counter to camera page
* Contributors: Christopher Berner

1.0.3 (2015-08-23)
------------------
* Fix race freeing background image sink resources
* Fix bad free on older versions of libav
* Improve formatting of first_time_setup.sh output
* Update readme
* Contributors: Christopher Berner

1.0.2 (2015-08-12)
------------------
* Remove unnecessary imports
* Contributors: Christopher Berner

1.0.1 (2015-08-09)
------------------
* Fix compilation error on vivid and utopic
* Contributors: Christopher Berner

1.0.0 (2015-08-08)
------------------
* Switch to H264 codec for streaming and recording
* Fix a variety of minor bugs
* Contributors: Christopher Berner

0.2.5 (2015-05-31)
------------------
* Tune MFC encoder parameters
* Contributors: Christopher Berner

0.2.4 (2015-05-29)
------------------
* Fix hardware encoder
* Fix detection of MFC
* Update service script for jade
* Contributors: Christopher Berner

0.2.3 (2015-05-02)
------------------
* Fix deprecation warnings
* Contributors: Christopher Berner

0.2.2 (2015-04-30)
------------------
* Change CodecID to AVCodecID
* Contributors: Christopher Berner

0.2.1 (2015-04-29)
------------------
* Switch to libnl 3.0
* Update to new libavconv constants
* Improve error handling in setup script
* Contributors: Christopher Berner

0.2.0 (2015-04-22)
------------------
Main features:

* Added map server to replace Google Maps
* Added support for recording in h264 with hardware acceleration
* Added support for Odroid Show

Details:

* Fix restart command in init.d script
* Move services and params out of global namespace
* Fix loading of video device selector
* Add hostapd setup to first_time_setup script
* Add setting in UI page to change codec
* Fix packaging of camera_node
* Optimize memory access when using MFC encoder
* Fix mjpeg recording
* Remove usage of tempnam, and cleanup some other code
* Fix usage of avcodec_encode_video2
* Replace usage of deprecated function
* Split camera node source code into cpp files
* Add (experimental) support for Exynos MFC
* Fix h264 encoding
* Improve media path expansion
* Fix bug in auto resolution detection
* Add auto-adjustment of height and width
* Wait for device to connect before writing to Odroid Show
* More code cleanup
* Code cleanup
* Fix debian package build
* Add support for recording with mjpeg codec
* Remove old vlc recorder node
* Add recording in h264 support to camera node
* Add support for usb cameras to camera node
* Refactor ptp node
* Media improvements
  Fix container format of recorded videos
  Add button to delete media
* Add support for Odroid Show
* Add thumbnails for videos
* Set queue_size in mavlink node
* Add flight mode to BasicStatus message and web ui
* Fix race condition in settings page
* Add carto style to osm2pgsql command
* Add auto detection of APM and baudrate
* Implement local mapnik server
  Also remove our dependency on Google Maps, so that we can run the map
  even when there's no internet connection
* Rename variable to avoid shadowing
* Fix image capture from webcam
* Fix lint errors
* Improve internet connection detection logic
* Don't try to load google maps if there's no internet connection
* Reduce chart update rate to 2Hz to improve performance
* Add source maps for Angular and jQuery
* Add button to shutdown on-board computer
* Contributors: Christopher Berner

0.1.1 (2014-08-27)
------------------
* Fix debian package build
* Contributors: Christopher Berner

0.1.0 (2014-08-26)
------------------
* Add PTP support
* Add init.d script to auto start rospilot
* Contributors: Christopher Berner

0.0.4 (2014-07-05)
------------------
* Use more standard compliant glob syntax
* Make .gitignore less aggressive
* Contributors: Christopher Berner

0.0.3 (2014-06-28)
------------------
* Change web_ui to use pkg_resources for static assets
* Add udev rule to installation targets
* Contributors: Christopher Berner

0.0.2 (2014-06-15)
------------------
* Remove pymavlink dependency
* Switch to a library for the HMC5883 communication
* Add more documentation
* Add support for MPU6050 to firmware
* Contributors: Christopher Berner

0.0.1 (2014-06-02)
------------------
* Initial release of rospilot
* Contributors: Christopher Berner, bordicon, cberner
