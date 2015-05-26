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

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/CompressedImage.h>

#include<ptp.h>
#include<usb_camera.h>
#include<video_recorder.h>
#include<transcoders.h>

extern "C" {
#include <linux/videodev2.h>
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
    H264Encoder *h264Encoder = nullptr;
    SoftwareVideoRecorder *videoRecorder = nullptr;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;
    ros::ServiceServer startRecordServiceServer;
    ros::ServiceServer stopRecordServiceServer;

    std::string videoDevice;
    std::string codec; // "mjpeg" or "h264"
    std::string mfcPath;
    AVCodecID codecId;
    std::string mediaPath;
    int width;
    int height;
    int framerate;

private:
    bool sendPreview()
    {
        static time_point<high_resolution_clock> sixtyFramesAgo = high_resolution_clock::now();
        static int frameRateCounter = 0;

        sensor_msgs::CompressedImage image;
        if(camera != nullptr && camera->getLiveImage(&image)) {
            bool keyFrame = false;
            bool transcodedSuccessfully = false;
            if (codec == "mjpeg") {
                keyFrame = true;
                transcodedSuccessfully = true;
            }
            imagePub.publish(image);
            if (codec == "h264" && image.format == "jpeg") {
                jpegDecoder->decodeInPlace(&image);
                transcodedSuccessfully = h264Encoder->encodeInPlace(&image, 
                        &keyFrame);
            }
            if (videoRecorder != nullptr && transcodedSuccessfully) {
                videoRecorder->writeFrame(&image, keyFrame);
            }

            frameRateCounter++;
            if (frameRateCounter >= 60) {
                time_point<high_resolution_clock> currentTime = high_resolution_clock::now();
                duration<double> duration = (currentTime - sixtyFramesAgo);
                //ROS_INFO("Camera frame rate %f", frameRateCounter / duration.count());
                frameRateCounter = 0;
                sixtyFramesAgo = currentTime;
            }
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
        PixelFormat pixelFormat;
        if (codec == "h264") {
            pixelFormat = PIX_FMT_YUV420P;
        }
        else if (codec == "mjpeg") {
            // TODO: Do we need to detect this dynamically?
            // Different cameras might be 4:2:0, 4:2:2, or 4:4:4
            pixelFormat = PIX_FMT_YUVJ422P;
        }

        std::string videoDevice;
        node.param("video_device", videoDevice, std::string("/dev/video0"));

        if (jpegDecoder != nullptr) {
            delete jpegDecoder;
        }
        jpegDecoder = new JpegDecoder(width, height, pixelFormat);
        if (h264Encoder != nullptr) {
            delete h264Encoder;
        }
        h264Encoder = createEncoder();
        if (videoRecorder != nullptr) {
            delete videoRecorder;
        }
        ROS_INFO("Recording in %s", codec.c_str());
        videoRecorder = new SoftwareVideoRecorder(width, height, pixelFormat, codecId);
    }

    BaseCamera *createCamera()
    {
        std::string cameraType;
        node.param("camera_type", cameraType, std::string("usb"));

        node.param("video_device", videoDevice, std::string("/dev/video0"));
        node.param("image_width", width, 1920);
        node.param("image_height", height, 1080);
        node.param("framerate", framerate, 30);
        node.param("codec", codec, std::string("mjpeg"));
        if (codec == "h264") {
            codecId = AV_CODEC_ID_H264;
        }
        else if (codec == "mjpeg") {
            codecId = AV_CODEC_ID_MJPEG;
        }
        else {
            ROS_FATAL("Unknown codec: %s", codec.c_str());
        }
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
            ROS_INFO("Requesting camera res %dx%d", width, height);
            UsbCamera *camera = new UsbCamera(videoDevice, width, height, framerate);
            // Read the width and height, since the camera may have altered it to
            // something it supports
            width = camera->getWidth();
            height = camera->getHeight();
            ROS_INFO("Camera selected res %dx%d", width, height);
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

    H264Encoder *createEncoder()
    {
        if (mfcPath.size() > 0) {
            ROS_INFO("Using hardware encoder");
            return new ExynosMultiFormatCodecH264Encoder(mfcPath, width, height);
        }
        else {
            ROS_INFO("Using software encoder");
            return new SoftwareH264Encoder(width, height);
        }
    }

    ~CameraNode()
    {
        if (camera != nullptr) {
            delete camera;
        }
        delete jpegDecoder;
        delete h264Encoder;
        delete videoRecorder;
    }

    bool spin()
    {
        ROS_INFO("camera node is running.");
                
        while (node.ok())
        {
            // Process any pending service callbacks
            ros::spinOnce();
            std::string newVideoDevice;
            node.getParam("video_device", newVideoDevice);
            std::string newCodec;
            node.param("codec", newCodec, std::string("mjpeg"));
            if (newVideoDevice != videoDevice || newCodec != codec) {
                initCameraAndEncoders();
            }
            if(!sendPreview()) {
                // Sleep and hope the camera recovers
                usleep(1000*1000);
            }
            // Run at 1kHz
            usleep(1000);
        }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    CameraNode a;
    a.spin();
    return 0;
}
