#ifndef D400_CAMERA_INTERFACE_HPP
#define D400_CAMERA_INTERFACE_HPP

#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace vision
{

class CameraPoseData
{
public:
    float x_translation_;
    float y_translation_;
    float z_translation_;

    float x_velocity_;
    float y_velocity_;
    float z_velocity_;

    float x_acceleration_;
    float y_acceleration_;
    float z_acceleration_;

    float Qi_rotation_;
    float Qj_rotation_;
    float Qk_rotation_;
    float Qr_rotation_;

    float x_angular_velocity_;
    float y_angular_velocity_;
    float z_angular_velocity_;

    float x_angular_acceleration_;
    float y_angular_acceleration_;
    float z_angular_acceleration_;

    float tracker_confidence_;
    float mapper_confidence_;
};

class RealsenseInterface
{
public:
    RealsenseInterface() = delete;

    explicit RealsenseInterface(YAML::Node config);

    cv::Mat getRGB() const;

    cv::Mat getDepth() const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudBasic();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMappedPointCloud();

    void disableAutoExposure();

    bool loadNext();

    RealsenseInterface& operator++();

    vision::CameraPoseData getPoseData();

    const void* getRawDepth() const;
    const void* getRawColor() const;

    int getStreamWidth() const;
    int getStreamHeight() const;

    uint64_t getUTime() const;

private:
    bool enabled_;
    bool publish_pos_;
    std::string serial_;
    int width_;
    int height_;
    int fps_;

    const uint16_t* depth_image_;
    const uint16_t* color_image_;

    bool has_new_cloud_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud_;

    void compute_clouds();

    uint64_t utime_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::frameset frames_;
    rs2::frame unaligned_rgb_frame_;
    rs2::frame unaligned_depth_frame_;
    rs2::frame pose_frame_;
    rs2_intrinsics depth_intrinsics_;
    rs2_extrinsics depth_to_color_;
    rs2_intrinsics color_intrinsics_;
    float scale_;

    rs2::sensor sensor_color_;
    rs2::sensor sensor_depth_;
};
} 

#endif
