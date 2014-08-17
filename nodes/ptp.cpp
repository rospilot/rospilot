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
#include<iostream>
#include<gphoto2/gphoto2.h>
#include<gphoto2/gphoto2-context.h>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>

class PtpNode
{
private:
    ros::NodeHandle node;
    // NOTE: We don't need to guard these with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    GPContext *context;
    CameraFile *cameraFile;
    Camera *camera;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;

private:
    void checkErrorCode(int errorCode, std::string message)
    {
        if (errorCode != GP_OK) {
            ROS_FATAL("Fatal error (%d) in call to libgphoto2: %s", errorCode,
                    message.c_str());
            exit(EXIT_FAILURE);
        }
    }

    bool sendPreview()
    {
        // Read from camera
        int errorCode = gp_camera_capture_preview(camera, cameraFile, context);
        if (errorCode != GP_OK) {
            ROS_WARN("Got error %d from gp_camera_capture_preview", errorCode);
            return false;
        }

        sensor_msgs::CompressedImage image;
        if(makeImage(cameraFile, &image)) {
            imagePub.publish(image);
        }
        return true;
    }

    bool makeImage(CameraFile *file, sensor_msgs::CompressedImage *image)
    {
        image->format = "jpeg";
        // Check mime type
        const char *mimeType;
        int errorCode = gp_file_get_mime_type(file, &mimeType);
        if (errorCode != GP_OK) {
            ROS_WARN("Got error %d from gp_file_get_mime_type", errorCode);
            return false;
        }
        if (strcmp(mimeType, GP_MIME_JPEG) != 0) {
            ROS_WARN("ptp node only designed to handle jpegs, got %s", 
                    mimeType);
        }

        // Extract data
        const char *data;
        unsigned long int size;
        errorCode = gp_file_get_data_and_size(file, &data, &size);
        if (errorCode != GP_OK) {
            ROS_WARN("Got error %d from gp_file_get_data_and_size", errorCode);
            return false;
        }
        for (unsigned long int i = 0; i < size; i++) {
            image->data.push_back(data[i]);
        }

        image->header.stamp = ros::Time::now();
        return true;
    }

public:
    PtpNode() : node("camera") 
    {
        imagePub = node.advertise<sensor_msgs::CompressedImage>(
                "image_raw/compressed", 1);
        captureServiceServer = node.advertiseService(
                "capture_image", 
                &PtpNode::captureImageCallback,
                this);

        context = gp_context_new();
        checkErrorCode(gp_camera_new(&camera), "gp_camera_new");
        int errorCode = gp_camera_init(camera, context);
        if (errorCode == GP_ERROR_IO_USB_CLAIM) {
            ROS_FATAL("Unable to claim USB device. Is it mounted "
                    "or in use by another process?");
            exit(EXIT_FAILURE);
        } else if (errorCode == GP_ERROR_MODEL_NOT_FOUND) {
            ROS_FATAL("PTP Camera not found. Is it powered on and "
                    "correctly configured?");
            exit(EXIT_FAILURE);
        }
        checkErrorCode(errorCode, "gp_camera_init");
        checkErrorCode(gp_file_new(&cameraFile), "gp_file_new");

        // Needed to make Canon cameras work
        CameraWidget *config;
        checkErrorCode(gp_camera_get_config(camera, &config, context), 
                "gp_camera_get_config");
        CameraWidget *child;
        checkErrorCode(gp_widget_get_child_by_name(config, "capture", &child), 
                "gp_widget_get_child_by_name");
        int on = 1;
        checkErrorCode(gp_widget_set_value(child, &on), "gp_widget_set_value");
        checkErrorCode(gp_camera_set_config(camera, config, context),
                "gp_camera_set_config");
    }

    ~PtpNode()
    {
        gp_camera_exit(camera, context);
        gp_camera_unref(camera);
        gp_context_unref(context);
        gp_file_free(cameraFile);
    }

    bool spin()
    {
        ROS_INFO("ptp node is running.");
                
        while (node.ok())
        {
            // Process any pending service callbacks
            ros::spinOnce();
            if(!sendPreview()) {
                // Sleep and hope the camera recovers
                usleep(1000*1000);
            }
            // Run at 1kHz
            usleep(1000);
        }
        return true;
    }

    bool captureImageCallback(rospilot::CaptureImage::Request& request, 
            rospilot::CaptureImage::Response &response)
    {
        // Read from camera
        CameraFilePath filePath;
        ROS_INFO("capturing");
        int errorCode = gp_camera_capture(
                camera,
                GP_CAPTURE_IMAGE,
                &filePath,
                context);
        if (errorCode != GP_OK) {
            ROS_WARN("Got error %d from gp_camera_capture", errorCode);
            return false;
        }
        ROS_INFO("captured");

        errorCode = gp_camera_file_get(
                camera,
                filePath.folder,
                filePath.name,
                GP_FILE_TYPE_NORMAL,
                cameraFile,
                context);
        
        if (errorCode != GP_OK) {
            ROS_WARN("Got error %d from gp_camera_file_get", errorCode);
            return false;
        }
        ROS_INFO("sending");

        return makeImage(cameraFile, &response.image);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptp");
    PtpNode a;
    a.spin();
    return 0;
}
