#include <fstream>
#include <stdexcept>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <kinect/depth/polynomial_matrix_io.h>

#include "depth_correction.h"

namespace depth_correction
{

void DepthCorrectionNodelet::onInit()
{
    _imageTransport.reset(new image_transport::ImageTransport(getNodeHandle()));

    try {
        loadParameters();
        advertise();
    } catch (std::exception &e) {
        NODELET_ERROR("Could not start correct_depth nodelet: %s", e.what());
        throw;
    }
}

void DepthCorrectionNodelet::loadParameters()
{
    ros::NodeHandle &nh = getPrivateNodeHandle();
    std::string globalUndMatrixFileName = nh.param("global_und_matrix", std::string("")),
            localUndMatrixFileName = nh.param("local_und_matrix", std::string(""));

    if (globalUndMatrixFileName.empty())
        throw std::invalid_argument("Missing global_und_matrix parameter");

    if (localUndMatrixFileName.empty())
        throw std::invalid_argument("Missing local_und_matrix parameter");

    int imageWidth, imageHeight;
    nh.param(std::string("depth_image/cols"), imageWidth, 640);
    nh.param(std::string("depth_image/rows"), imageHeight, 480);
    _imagesSize.x() = imageWidth;
    _imagesSize.y() = imageHeight;

    calibration::LocalModel::Data::Ptr localUndData;
    calibration::PolynomialUndistortionMatrixIO<calibration::LocalPolynomial> local_io;
    if (not local_io.read(localUndData, localUndMatrixFileName))
        NODELET_FATAL_STREAM("Cannot read " << localUndMatrixFileName);

    calibration::GlobalModel::Data::Ptr globalUndData;
    calibration::PolynomialUndistortionMatrixIO<calibration::GlobalPolynomial> global_io;
    if (not global_io.read(globalUndData, globalUndMatrixFileName))
        NODELET_FATAL_STREAM("Cannot read " << globalUndMatrixFileName);

    calibration::LocalModel::Ptr local_model = boost::make_shared<calibration::LocalModel>(_imagesSize);
    local_model->setMatrix(localUndData);

    calibration::GlobalModel::Ptr global_model = boost::make_shared<calibration::GlobalModel>(_imagesSize);
    global_model->setMatrix(globalUndData);

    calibration::UndistortionModel::Ptr model = boost::make_shared<calibration::UndistortionModel>();
    model->setLocalModel(local_model);
    model->setGlobalModel(global_model);

    _sensor.setUndistortionModel(model);
}

void DepthCorrectionNodelet::advertise()
{
    std::lock_guard<std::mutex> lock(_connectMutex);

    _imagePub = _imageTransport->advertise("image_rect", 1,
                                           boost::bind(&DepthCorrectionNodelet::onConnect, this),
                                           boost::bind(&DepthCorrectionNodelet::onDisconnect, this));
}

void DepthCorrectionNodelet::onConnect()
{
    std::lock_guard<std::mutex> lock(_connectMutex);

    if (!_imageSub)
    {
        NODELET_DEBUG("Begin depth correction.");

        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        _imageSub = _imageTransport->subscribe("image", 1, &DepthCorrectionNodelet::onDepthImage, this, hints);
    }
}

void DepthCorrectionNodelet::onDisconnect()
{
    std::lock_guard<std::mutex> lock(_connectMutex);

    if (_imagePub.getNumSubscribers() == 0)
    {
        NODELET_DEBUG("No more subscriber, end depth correction.");

        _imageSub.shutdown();
    }
}

void DepthCorrectionNodelet::onDepthImage(const sensor_msgs::ImageConstPtr &rawImage)
{
    if (rawImage->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
        NODELET_ERROR_STREAM("Unsupported image encoding " << rawImage->encoding);
        return;
    }

    sensor_msgs::ImagePtr newImage = boost::make_shared<sensor_msgs::Image>();
    newImage->header = rawImage->header;
    newImage->height = rawImage->height;
    newImage->width = rawImage->width;
    newImage->encoding = rawImage->encoding;
    newImage->step = rawImage->step;
    newImage->data.resize(newImage->step * newImage->height);

    auto* uncorrectedData = reinterpret_cast<const float *>(rawImage->data.data());
    auto* correctedData = reinterpret_cast<float *>(newImage->data.data());
    const size_t width = rawImage->width;
    const size_t height = rawImage->height;

    for (size_t y = 0; y < height; ++y)
    {
        for (size_t x = 0; x < width; ++x)
        {
            double z = uncorrectedData[y * width + x];
            _sensor.undistortionModel()->localModel()->undistort(x, y, z);
            _sensor.undistortionModel()->globalModel()->undistort(x, y, z);
            correctedData[y * width + x] = z;
        }
    }

    _imagePub.publish(newImage);
}

} // namespace depth_correction.

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_correction::DepthCorrectionNodelet, nodelet::Nodelet);
