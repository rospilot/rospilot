^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospilot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
