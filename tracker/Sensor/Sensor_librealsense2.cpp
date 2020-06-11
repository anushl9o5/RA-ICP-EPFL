#include "Sensor.h"
#include "tracker/Types.h"

#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "util/Sleeper.h"
#include "util/tictoc.h"


#ifndef HAS_LIBREALSENSE2
    SensorLibRealSense2::SensorLibRealSense2(Camera *camera) : Sensor(camera){ mFatal() << "Intel LibRealSense2 not available in your OS"; }
    int SensorLibRealSense2::initialize(){ return 0; }
    SensorLibRealSense2::~SensorLibRealSense2(){}
    bool SensorLibRealSense2::spin_wait_for_data(Scalar timeout_seconds){ return false; }
    bool SensorLibRealSense2::fetch_streams(DataFrame &frame){ return false; }
    void SensorLibRealSense2::start(){}
    void SensorLibRealSense2::stop(){}
#else

#include "Sensor.h"

#include <stdio.h>
#include <vector>
#include <exception>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <QElapsedTimer>
#include <QApplication>
#include <QMessageBox>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/Camera.h"

using namespace std;


rs2::config cfg;
rs2::context ctx;
rs2::pipeline pipe(ctx);
rs2_intrinsics depth_intrin, color_intrin;
rs2_extrinsics depth_to_color;
rs2::stream_profile depth_stream;
rs2::stream_profile color_stream;


int D_width  = 640;
int D_height = 480;
float scale;

SensorLibRealSense2::SensorLibRealSense2(Camera *camera) : Sensor(camera) {
    if (camera->mode() != Intel)
        LOG(FATAL) << "!!!FATAL: LibRealSense needs Intel camera mode";
}

int SensorLibRealSense2::initialize() {
    std::cout << "SensorLibRealSense::initialize()" << std::endl;

    rs2::device_list devs = ctx.query_devices();
    if (devs.size() == 0) return EXIT_FAILURE;

    rs2::device dev = devs[0];
    printf("\nUsing device 0, an %s\n", dev.get_info(RS2_CAMERA_INFO_NAME));
    printf("    Serial number: %s\n", dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    printf("    Firmware version: %s\n", dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));

    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_DEPTH, D_width, D_height, RS2_FORMAT_Z16, 24);
    cfg.enable_stream(RS2_STREAM_COLOR, D_width, D_height, RS2_FORMAT_RGB8, 24);

    printf("Enabled Streams:Depth and Color\n");

    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    printf("Device Started\n");

    depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH);
    color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);

    depth_intrin = depth_stream.as<rs2::video_stream_profile>().get_intrinsics();
    color_intrin = color_stream.as<rs2::video_stream_profile>().get_intrinsics();
    depth_to_color = depth_stream.get_extrinsics_to(color_stream);
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            scale = dpt.get_depth_scale();
        }
    }

    //depth_intrin = dev->get_stream_intrinsics(rs2::stream::depth);
    //depth_to_color = dev->get_extrinsics(rs2::stream::depth, rs2::stream::color);
    //color_intrin = dev->get_stream_intrinsics(rs2::stream::color);
    //scale = dev->get_depth_scale();

    this->initialized = true;
    return true;
}

SensorLibRealSense2::~SensorLibRealSense2() {
    std::cout << "~SensorLibRealSense()" << std::endl;

    if (!initialized) return;
    // TODO: stop sensor

}

bool SensorLibRealSense2::spin_wait_for_data(Scalar timeout_seconds) {

    DataFrame frame(-1);
    QElapsedTimer chrono;
    chrono.start();
    while (fetch_streams(frame) == false) {
        LOG(INFO) << "Waiting for data.. " << chrono.elapsed();
        Sleeper::msleep(500);
        QApplication::processEvents(QEventLoop::AllEvents);
        if (chrono.elapsed() > 1000 * timeout_seconds)
            return false;
    }
    return true;
}

bool SensorLibRealSense2::fetch_streams(DataFrame &frame) {
    if (initialized == false) this->initialize();

    rs2::frameset frames;

    if(frame.depth.empty())
        frame.depth = cv::Mat(cv::Size(D_width/2, D_height/2), CV_16UC1, cv::Scalar(0));
    if(frame.color.empty())
        frame.color = cv::Mat(cv::Size(D_width/2, D_height/2), CV_8UC3, cv::Scalar(0, 0, 0));

    pipe.poll_for_frames(&frames);
    const uint16_t * depth_image = (const uint16_t *)frames.get_depth_frame().get_data();
    const uint8_t * color_image = (const uint8_t *)frames.get_color_frame().get_data();



    cv::Mat depth_buffer = cv::Mat(cv::Size(D_width/2, D_height/2), CV_16UC1, cv::Scalar(0));
    cv::Mat color_buffer = cv::Mat(cv::Size(D_width/2, D_height/2), CV_8UC3, cv::Scalar(255,255,255));
    for(int dy=0,dy_sub=0; dy<depth_intrin.height; dy+=2,dy_sub++)
    {
        for(int dx=0,dx_sub=0; dx<depth_intrin.width; dx+=2,dx_sub++)
        {
            uint16_t depth_value = depth_image[dy * depth_intrin.width + (depth_intrin.width-dx-1)];
            float depth_in_meters = depth_value * scale;
            float pixel_depth_in_mm = depth_in_meters * 1000;
            if(depth_value == 0) continue;

            float depth_pixel[2] = {(float)(depth_intrin.width-dx-1), (float)dy};

            float depth_point[3];
            rs2_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth_in_meters);
            float color_point[3];
            rs2_transform_point_to_point(depth_point, &depth_to_color, color_point);
            float color_pixel[2];
            rs2_project_point_to_pixel(color_pixel, &color_intrin, depth_point);

            const int cx = (int)std::round(color_pixel[0]), cy = (int)std::round(color_pixel[1]);
            if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
            {
                color_buffer.at<cv::Vec3b>(dy_sub, dx_sub) = cv::Vec3b(255,255,255);
            }
            else
            {
                unsigned char r = color_image[cy * D_width * 3 + (cx) * 3 + 0];
                unsigned char g = color_image[cy * D_width * 3 + (cx) * 3 + 1];
                unsigned char b = color_image[cy * D_width * 3 + (cx) * 3 + 2];
                color_buffer.at<cv::Vec3b>(dy_sub, dx_sub) = cv::Vec3b(r,g,b);
                depth_buffer.at<unsigned short>(dy_sub,dx_sub) = (unsigned short)pixel_depth_in_mm;
            }
        }
    }
    frame.color = color_buffer;
    frame.depth = depth_buffer;
    return true;
}

void SensorLibRealSense2::start() {
    if (!initialized)
        this->initialize();
}

void SensorLibRealSense2::stop() {
}
#endif
