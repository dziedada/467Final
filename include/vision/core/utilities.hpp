#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <common/messages/depth_image_t.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <common/messages/point_cloud_t.hpp>

#include <memory>

namespace vision
{
cv::Mat wrapInRGBMat(void* mem_ptr, int width, int height);

cv::Mat wrapInDepthMat(void* mem_ptr, int width, int height);

void rgbdToZcmType(const cv::Mat& rgb, const cv::Mat& depth, rgbd_image_t& zcm_img);

void zcmTypeToRgbd(const rgbd_image_t& zcm_img, cv::Mat& rgb, cv::Mat& depth);

pcl::PointCloud<pcl::PointXYZ>::Ptr zcmTypeToPCLPointCloud(const point_cloud_t& zcm_cloud);
}
