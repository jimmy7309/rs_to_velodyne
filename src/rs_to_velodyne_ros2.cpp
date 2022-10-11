//#include "utility.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
                                                     uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D

  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(uint16_t, ring, ring)(float, time,
                                                                                                           time))
#if 0
struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)
#endif

class RsToVelodyne : public rclcpp::Node
{
public:
  RsToVelodyne() : Node("ros_to_velodyne_ros2")
  {
    using std::placeholders::_1;

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/points", 10, std::bind(&RsToVelodyne::rsHandler_XYZIRT, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/points", 10);
  }

private:
  void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) const
  {
    pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(*pc_msg, *pc_in);

    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
    handle_pc_msg(pc_in, pc_out);
    add_ring(pc_in, pc_out);
    add_time(pc_in, pc_out);

    this->publish_points(pc_out, pc_msg);
  }

  void publish_points(pcl::PointCloud<VelodynePointXYZIRT>::Ptr new_pc,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr old_msg) const
  {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg->header;
    pc_new_msg.header.frame_id = "velodyne";
    publisher_->publish(pc_new_msg);
  }

  bool has_nan(RsPointXYZIRT point) const
  {
    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    // pcl remove nan not work normally
    // ROS_ERROR("Containing nan point!");
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void handle_pc_msg(const pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in,
                     const pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out) const
  {
    // to new pointcloud
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
      if (has_nan(pc_in->points[point_id]))
        continue;
      VelodynePointXYZIRT new_point;
      // std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
      new_point.x = pc_in->points[point_id].x;
      new_point.y = pc_in->points[point_id].y;
      new_point.z = pc_in->points[point_id].z;
      new_point.intensity = pc_in->points[point_id].intensity;
      // new_point.ring = pc->points[point_id].ring;
      // // 计算相对于第一个点的相对时间
      // new_point.time = float(pc->points[point_id].timestamp - pc->points[0].timestamp);
      pc_out->points.push_back(new_point);
    }
  }

  void add_ring(const pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in,
                const pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out) const
  {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
      if (has_nan(pc_in->points[point_id]))
        continue;
      // 跳过nan点
      pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
  }

  void add_time(const pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in,
                const pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out) const
  {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
      if (has_nan(pc_in->points[point_id]))
        continue;
      // 跳过nan点
      pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RsToVelodyne>());
  rclcpp::shutdown();
  return 0;
}
