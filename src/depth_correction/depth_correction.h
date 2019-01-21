#ifndef PROJECT_RGBD_CORRECT_NODE_H
#define PROJECT_RGBD_CORRECT_NODE_H

#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <rgbd_calibration/globals.h>
#include <kinect/depth/sensor.h>


namespace depth_correction
{

/**
 * \brief Subscribe to "image", a depth image with the 32FC1 encoding
 * and produce a 32FC1 encoded image "image_rect" of the rectified depth
 */
class DepthCorrectionNodelet: public nodelet::Nodelet
{
public:
    ~DepthCorrectionNodelet() override = default;

    void onInit() override;

private:
    void onConnect();
    void onDisconnect();
    void onDepthImage(const sensor_msgs::ImageConstPtr &raw_msg);

    void loadParameters();
    void advertise();

private:
    std::mutex _connectMutex;

    std::unique_ptr<image_transport::ImageTransport> _imageTransport;

    image_transport::Publisher _imagePub;
    image_transport::Subscriber _imageSub;

    calibration::Size2 _imagesSize;
    calibration::KinectDepthSensor<calibration::UndistortionModel> _sensor;
};

} // namespace depth_correction.

#endif //PROJECT_RGBD_CORRECT_NODE_H
