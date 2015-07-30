/*********************************************************************
 *
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/
#include<stdio.h>
#include<time.h>
#include<fstream>
#include<chrono>
#include<boost/filesystem.hpp>
#include<wordexp.h>
#include<dirent.h>
#include<sys/ioctl.h>
#include<fcntl.h>
#include<errno.h>
#include<mutex>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<rospilot/Resolutions.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/CompressedImage.h>

#include<background_image_sink.h>
#include<ptp.h>
#include<usb_camera.h>
#include<video_recorder.h>
#include<transcoders.h>
#include<resizer.h>
#include<h264_server.h>

extern "C" {
#include <linux/videodev2.h>
}

namespace rospilot {

static int ffmpegLockManager(void **mtx, enum AVLockOp op)
{
 switch(op) {
 case AV_LOCK_CREATE:
   *mtx = new std::mutex();
   if(!*mtx)
     return 1;
   return 0;
 case AV_LOCK_OBTAIN:
   ((std::mutex *) *mtx)->lock();
   return 0;
 case AV_LOCK_RELEASE:
   ((std::mutex *) *mtx)->unlock();
   return 0;
 case AV_LOCK_DESTROY:
   delete (std::mutex *) *mtx;
   *mtx = nullptr;
   return 0;
 }
 return 1;
}

using namespace std::chrono;

class CameraNode
{
private:
    ros::NodeHandle node;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    BaseCamera *camera = nullptr;
    JpegDecoder *jpegDecoder = nullptr;
    SoftwareVideoRecorder *videoRecorder = nullptr;
    BackgroundImageSink *liveStream = nullptr;
    BackgroundImageSink *recorder = nullptr;
    H264Server h264Server;

    ros::Publisher resolutionsTopic;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;
    ros::ServiceServer startRecordServiceServer;
    ros::ServiceServer stopRecordServiceServer;

    std::string videoDevice;
    std::string mfcPath;
    std::string mediaPath;
    int cameraWidth;
    int cameraHeight;
    int framerate;

private:
    bool sendPreview()
    {
        sensor_msgs::CompressedImage image;
        if(camera != nullptr && camera->getLiveImage(&image)) {
            bool keyFrame = false;
            bool transcodedSuccessfully = false;
            imagePub.publish(image);
            jpegDecoder->decodeInPlace(&image);
            liveStream->addFrame(&image);
            recorder->addFrame(&image);

            return true;
        }
        return false;
    }

    void initCameraAndEncoders()
    {
        if (camera != nullptr) {
            delete camera;
        }
        camera = createCamera();
        resolutionsTopic.publish(camera->getSupportedResolutions());
        PixelFormat pixelFormat = PIX_FMT_YUV420P;

        std::string videoDevice;
        node.param("video_device", videoDevice, std::string("/dev/video0"));

        H264Settings recordingH264Settings;
        recordingH264Settings.height = cameraHeight;
        recordingH264Settings.width = cameraWidth;
        recordingH264Settings.level = 41;
        recordingH264Settings.gop_size = 30;
        recordingH264Settings.zero_latency = false;
        recordingH264Settings.profile = HIGH;
        recordingH264Settings.height = cameraHeight;
        recordingH264Settings.width = cameraWidth;
        recordingH264Settings.bit_rate = 4 * cameraWidth * cameraHeight;

        double aspectRatio = cameraHeight / (double) cameraWidth;
        H264Settings liveH264Settings;
        liveH264Settings.height = (int) (aspectRatio * 640);
        liveH264Settings.width = 640;
        liveH264Settings.level = 41;
        liveH264Settings.gop_size = 12;
        liveH264Settings.bit_rate = 1000 * 1000;
        liveH264Settings.zero_latency = true;
        liveH264Settings.profile = CONSTRAINED_BASELINE;

        if (jpegDecoder != nullptr) {
            delete jpegDecoder;
        }
        jpegDecoder = new JpegDecoder(cameraWidth, cameraHeight, pixelFormat);

        if (liveStream != nullptr) {
            delete liveStream;
        }
        Resizer *resizer = new Resizer(
                cameraWidth,
                cameraHeight,
                liveH264Settings.width,
                liveH264Settings.height,
                pixelFormat);
        liveStream = new BackgroundImageSink(
                &h264Server,
                createEncoder(liveH264Settings),
                resizer);

        if (videoRecorder != nullptr) {
            delete videoRecorder;
        }
        videoRecorder = new SoftwareVideoRecorder(pixelFormat, recordingH264Settings);
        if (recorder != nullptr) {
            delete recorder;
        }
        recorder = new BackgroundImageSink(
                videoRecorder,
                createEncoder(recordingH264Settings),
                nullptr
        );
    }

    BaseCamera *createCamera()
    {
        std::string cameraType;
        node.param("camera_type", cameraType, std::string("usb"));

        node.param("video_device", videoDevice, std::string("/dev/video0"));
        node.param("image_width", cameraWidth, 1920);
        node.param("image_height", cameraHeight, 1080);
        node.param("framerate", framerate, 30);
        node.param("media_path", mediaPath, std::string("~/.rospilot/media"));
        wordexp_t p;
        wordexp(mediaPath.c_str(), &p, 0);
        if (p.we_wordc != 1) {
            ROS_ERROR("Got too many words when expanding media path: %s",
                    mediaPath.c_str());
        }
        else {
            mediaPath = p.we_wordv[0];
        }
        wordfree(&p);


        if (cameraType == "ptp") {
            return new PtpCamera();
        }
        else if (cameraType == "usb") {
            ROS_INFO("Requesting camera res %dx%d", cameraWidth, cameraHeight);
            UsbCamera *camera = new UsbCamera(videoDevice, cameraWidth, cameraHeight, framerate);
            // Read the width and height, since the camera may have altered it to
            // something it supports
            cameraWidth = camera->getWidth();
            cameraHeight = camera->getHeight();
            node.setParam("image_width", cameraWidth);
            node.setParam("image_height", cameraHeight);
            ROS_INFO("Camera selected res %dx%d", cameraWidth, cameraHeight);
            return camera;
        }
        else {
            ROS_FATAL("Unsupported camera type: %s", cameraType.c_str());
            return nullptr;
        }
    }

public:
    CameraNode() : node("~") 
    {
        resolutionsTopic = node.advertise<rospilot::Resolutions>(
                "resolutions", 1, true);
        imagePub = node.advertise<sensor_msgs::CompressedImage>(
                "image_raw/compressed", 1);
        captureServiceServer = node.advertiseService(
                "capture_image", 
                &CameraNode::captureImageCallback,
                this);
        startRecordServiceServer = node.advertiseService(
                "start_record", 
                &CameraNode::startRecordHandler,
                this);
        stopRecordServiceServer = node.advertiseService(
                "stop_record", 
                &CameraNode::stopRecordHandler,
                this);
        mfcPath = findMfcDevice();

        // Install lock manager to make ffmpeg thread-safe
        if (av_lockmgr_register(ffmpegLockManager)) {
            ROS_FATAL("Failed to make ffmpeg thread-safe");
        }

        initCameraAndEncoders();
    }

    std::string findMfcDevice()
    {
        // Look for the MFC
        DIR *dir = opendir("/dev");
        dirent *dirEntry = nullptr;
        while ((dirEntry = readdir(dir)) != nullptr) {
            int fd;
            v4l2_capability videoCap;
            std::string path = std::string("/dev/") + dirEntry->d_name;
            if (path.substr(0, std::string("/dev/video").size()) != "/dev/video" ||
                    path == videoDevice) {
                continue;
            }
            ROS_INFO("Querying %s", path.c_str());
            if((fd = open(path.c_str(), O_RDONLY)) == -1){
                ROS_WARN("Can't open %s: %s", path.c_str(), strerror(errno));
                continue;
            }

            if(ioctl(fd, VIDIOC_QUERYCAP, &videoCap) == -1) {
                ROS_WARN("Can't read from %s: %s", path.c_str(), strerror(errno));
            }
            else {
                if (std::string((char *) videoCap.driver) == "s5p-mfc") {
                    v4l2_ext_controls ctrls;
                    v4l2_ext_control ctrl;

                    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
                    // These aren't used for integer controls
                    ctrl.size = 0;
                    memzero(ctrl.reserved2);

                    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
                    ctrls.count = 1;
                    ctrls.controls = &ctrl;
                    
                    if(ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0) {
                        close(fd);
                        return path;
                    }
                }
            }
            close(fd);
        }
        closedir(dir);
        return "";
    }

    H264Encoder *createEncoder(H264Settings settings)
    {
        if (mfcPath.size() > 0 && !settings.zero_latency) {
            ROS_INFO("Using hardware encoder");
            return new ExynosMultiFormatCodecH264Encoder(mfcPath, settings);
        }
        else {
            ROS_INFO("Using software encoder");
            return new SoftwareH264Encoder(settings);
        }
    }

    ~CameraNode()
    {
        if (camera != nullptr) {
            delete camera;
        }
        delete jpegDecoder;
        delete recorder;
        delete liveStream;
        delete videoRecorder;
        av_lockmgr_register(nullptr);
    }

    bool spin()
    {
        ROS_INFO("camera node is running.");
        h264Server.start();
        while (node.ok())
        {
            // Process any pending service callbacks
            ros::spinOnce();
            int newWidth;
            node.getParam("image_width", newWidth);
            int newHeight;
            node.getParam("image_height", newHeight);
            std::string newVideoDevice;
            node.getParam("video_device", newVideoDevice);
            if (newVideoDevice != videoDevice || 
                    newWidth != cameraWidth ||
                    newHeight != cameraHeight) {
                initCameraAndEncoders();
            }
            if(!sendPreview()) {
                // Sleep and hope the camera recovers
                usleep(1000*1000);
            }
            // Run at 1kHz
            usleep(1000);
        }
        h264Server.stop();
        return true;
    }
    
    bool startRecordHandler(std_srvs::Empty::Request& request, 
            std_srvs::Empty::Response &response)
    {
        time_t t = time(nullptr);
        struct tm *tmp = localtime(&t);
        char str[100];
        strftime(str, sizeof(str), "%Y-%m-%d_%H%M%S.mp4", tmp);
        std::string path = mediaPath + "/" + str;
        return videoRecorder->start(path.c_str());
    }
    
    bool stopRecordHandler(std_srvs::Empty::Request& request, 
            std_srvs::Empty::Response &response)
    {
        return videoRecorder->stop();
    }

    bool captureImageCallback(std_srvs::Empty::Request& request, 
            std_srvs::Empty::Response &response)
    {
        if (camera != nullptr) {
            time_t t = time(nullptr);
            struct tm *tmp = localtime(&t);
            char str[100];
            strftime(str, sizeof(str), "%Y-%m-%d_%H%M%S", tmp);
            std::string basePath = mediaPath + "/" + str;
            std::string path = basePath + ".jpg";
            for (int i = 1; boost::filesystem::exists(path); i++) {
                std::stringstream ss;
                ss << basePath << "_" << i << ".jpg";
                path = ss.str();
            }
            sensor_msgs::CompressedImage image;
            if (!camera->captureImage(&image)) {
                return false;
            }
            std::fstream fout(path, std::fstream::out);
            fout.write((char*) image.data.data(), image.data.size());
            fout.close();
            return true;
        }
        return false;
    }
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    rospilot::CameraNode a;
    a.spin();
    return 0;
}

