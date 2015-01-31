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
#include<unistd.h>
#include<pwd.h>
#include<iostream>
#include<gphoto2/gphoto2.h>
#include<gphoto2/gphoto2-context.h>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>

#include<ptp.h>
#include<usb_camera.h>
#include<video_recorder.h>

class CameraNode
{
private:
    ros::NodeHandle node;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    BaseCamera *camera;
    JpegDecoder *jpegDecoder;
    H264Encoder *h264Encoder;
    SoftwareVideoRecorder *videoRecorder;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;
    ros::ServiceServer startRecordServiceServer;
    ros::ServiceServer stopRecordServiceServer;

    std::string videoDevice;
    std::string codec; // "mjpeg" or "h264"
    CodecID codecId;
    std::string mediaPath;
    int width;
    int height;
    int framerate;

private:
    bool sendPreview()
    {
        sensor_msgs::CompressedImage image;
        if(camera != nullptr && camera->getLiveImage(&image)) {
            bool keyFrame = false;
            if (codec == "mjpeg") {
                keyFrame = true;
            }
            imagePub.publish(image);
            if (codec == "h264" && image.format == "mjpeg") {
                jpegDecoder->decodeInPlace(&image);
                h264Encoder->encodeInPlace(&image, &keyFrame);
            }
            if (videoRecorder != nullptr) {
                videoRecorder->writeFrame(&image, keyFrame);
            }
            return true;
        }
        return false;
    }

    // TODO: find a library that does this
    static std::string expandTildes(const std::string path)
    {
        if (path[0] != '~') {
            return path;
        }
        std::string user = "";
        int i = 1;
        for (; i < path.size() && path[i] != '/'; i++) {
            user += path[i];
        }
        if (user.size() == 0) {
            user = getpwuid(geteuid())->pw_name;
        }
        return std::string("/home/") + user + path.substr(i);
    }

    BaseCamera *createCamera()
    {
        std::string cameraType;
        node.param("camera_type", cameraType, std::string("usb"));

        node.param("video_device", videoDevice, std::string("/dev/video0"));
        node.param("image_width", width, 640);
        node.param("image_height", height, 480);
        node.param("framerate", framerate, 30);
        node.param("codec", codec, std::string("mjpeg"));
        if (codec == "h264") {
            codecId = CODEC_ID_H264;
        }
        else if (codec == "mjpeg") {
            codecId = CODEC_ID_MJPEG;
        }
        else {
            ROS_FATAL("Unknown codec: %s", codec.c_str());
        }
        node.param("media_path", mediaPath, std::string("~/.rospilot/media"));
        mediaPath = expandTildes(mediaPath);

        if (cameraType == "ptp") {
            return new PtpCamera();
        }
        else if (cameraType == "usb") {
            return new UsbCamera(videoDevice, width, height, framerate);
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
                "/start_record", 
                &CameraNode::startRecordHandler,
                this);
        stopRecordServiceServer = node.advertiseService(
                "/stop_record", 
                &CameraNode::stopRecordHandler,
                this);
        camera = createCamera();
        jpegDecoder = new JpegDecoder(width, height, PIX_FMT_YUV420P);
        h264Encoder = new SoftwareH264Encoder(width, height, PIX_FMT_YUV420P);
        PixelFormat recordingPixelFormat;
        if (codec == "h264") {
            recordingPixelFormat = PIX_FMT_YUV420P;
        }
        else if (codec == "mjpeg") {
            // TODO: Do we need to detect this dynamically?
            // Different cameras might be 4:2:0, 4:2:2, or 4:4:4
            recordingPixelFormat = PIX_FMT_YUVJ422P;
        }
        videoRecorder = new SoftwareVideoRecorder(width, height, recordingPixelFormat, codecId);
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
            if (newVideoDevice != videoDevice) {
                if (camera != nullptr) {
                    delete camera;
                }
                camera = createCamera();
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

    bool captureImageCallback(rospilot::CaptureImage::Request& request, 
            rospilot::CaptureImage::Response &response)
    {
        if (camera != nullptr) {
            return camera->captureImage(&response.image);
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
