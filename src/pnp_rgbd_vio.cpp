#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class PNPRGBDVIO : public rclcpp::Node
{
public:
  PNPRGBDVIO() : Node("pnp_rgbd_vio")
  {
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10,
        std::bind(&PNPRGBDVIO::rgbCallback, this, _1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth_image", 10,
        std::bind(&PNPRGBDVIO::depthCallback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 100,
        std::bind(&PNPRGBDVIO::imuCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    orb_ = cv::ORB::create(1500);

    RCLCPP_INFO(get_logger(), "PnP RGB-D VIO node started");
  }

private:
  // ---------------- IMU ----------------
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_orientation_ = msg->orientation;
    imu_ready_ = true;
  }

  // ---------------- DEPTH ----------------
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    depth_img_ = cv_bridge::toCvCopy(msg)->image;
  }

  // ---------------- RGB + PNP ----------------
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!imu_ready_ || depth_img_.empty())
      return;

    cv::Mat rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_->detectAndCompute(rgb, cv::noArray(), keypoints, descriptors);

    if (keypoints.size() < 20)
      return;

    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;

    for (auto &kp : keypoints)
    {
      int u = int(kp.pt.x);
      int v = int(kp.pt.y);

      if (u < 0 || v < 0 ||
          u >= depth_img_.cols || v >= depth_img_.rows)
        continue;

      float d = depth_img_.at<float>(v, u);
      if (d <= 0.3 || d > 5.0)
        continue;

      // Camera intrinsics (REPLACE WITH YOURS)
      float fx = 525, fy = 525;
      float cx = 319.5, cy = 239.5;

      float x = (u - cx) * d / fx;
      float y = (v - cy) * d / fy;
      float z = d;

      pts_3d.emplace_back(x, y, z);
      pts_2d.emplace_back(kp.pt);
    }

    if (pts_3d.size() < 10)
      return;

    cv::Mat rvec, tvec;
    cv::Mat K = (cv::Mat_<double>(3,3) <<
      525, 0, 319.5,
      0, 525, 239.5,
      0, 0, 1);

    cv::solvePnP(
        pts_3d, pts_2d,
        K, cv::Mat(),
        rvec, tvec,
        false, cv::SOLVEPNP_ITERATIVE);

    publishOdometry(tvec, msg->header.stamp);
  }

  // ---------------- OUTPUT ----------------
  void publishOdometry(const cv::Mat &tvec, const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = tvec.at<double>(0);
    odom.pose.pose.position.y = tvec.at<double>(1);
    odom.pose.pose.position.z = tvec.at<double>(2);

    odom.pose.pose.orientation = last_orientation_;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = last_orientation_;

    tf_broadcaster_->sendTransform(tf);
  }

  // ---------------- MEMBERS ----------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  cv::Ptr<cv::ORB> orb_;
  cv::Mat depth_img_;

  geometry_msgs::msg::Quaternion last_orientation_;
  bool imu_ready_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PNPRGBDVIO>());
  rclcpp::shutdown();
  return 0;
}
