#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "libobsensor/ObSensor.hpp"
#include "utils.hpp"

class PointCloudPublisher : public rclcpp::Node {
 public:
  PointCloudPublisher() : Node("point_cloud_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
  }
  void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "sensor_frame";
    publisher_->publish(output);
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

void createPointCloud(std::shared_ptr<ob::Frame> frame,
                      std::shared_ptr<PointCloudPublisher> publisher) {
  int pointsSize = frame->dataSize() / sizeof(OBPoint);
  OBPoint *point = (OBPoint *)frame->data();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = pointsSize;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  for (int i = 0; i < pointsSize; i++) {
    cloud->points[i].x = static_cast<float>(point->x) * 0.001f;
    cloud->points[i].y = static_cast<float>(point->y) * 0.001f;
    cloud->points[i].z = static_cast<float>(point->z) * 0.001f;
    point++;
  }
  publisher->publishPointCloud(cloud);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudPublisher>();

  ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
  ob::Pipeline pipeline;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
  try {
    auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
    if (colorProfiles) {
      auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
      colorProfile = profile->as<ob::VideoStreamProfile>();
    }
    config->enableStream(colorProfile);
  } catch (ob::Error &e) {
    config->setAlignMode(ALIGN_DISABLE);
    std::cerr << "Current device is not support color sensor!" << std::endl;
  }
  std::shared_ptr<ob::StreamProfileList> depthProfileList;
  OBAlignMode alignMode = ALIGN_DISABLE;
  if (colorProfile) {
    depthProfileList =
        pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
    if (depthProfileList->count() > 0) {
      alignMode = ALIGN_D2C_HW_MODE;
    } else {
      depthProfileList =
          pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
      if (depthProfileList->count() > 0) {
        alignMode = ALIGN_D2C_SW_MODE;
      }
    }
  } else {
    depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
  }

  if (depthProfileList->count() > 0) {
    std::shared_ptr<ob::StreamProfile> depthProfile;
    try {
      if (colorProfile) {
        depthProfile = depthProfileList->getVideoStreamProfile(
            OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
      }
    } catch (...) {
      depthProfile = nullptr;
    }

    if (!depthProfile) {
      depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
    }
    config->enableStream(depthProfile);
  }
  config->setAlignMode(alignMode);

  pipeline.start(config);
  ob::PointCloudFilter pointCloud;
  auto cameraParam = pipeline.getCameraParam();
  pointCloud.setCameraParam(cameraParam);

  while (rclcpp::ok()) {
    auto frameset = pipeline.waitForFrames(100);
    if (frameset != nullptr && frameset->depthFrame() != nullptr) {
      auto depthValueScale = frameset->depthFrame()->getValueScale();
      pointCloud.setPositionDataScaled(depthValueScale);
      try {
        pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
        std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
        createPointCloud(frame, node);
      } catch (std::exception &e) {
        std::cout << "Get point cloud failed" << std::endl;
      }
    }
  }

  pipeline.stop();
  rclcpp::shutdown();
  return 0;
}
