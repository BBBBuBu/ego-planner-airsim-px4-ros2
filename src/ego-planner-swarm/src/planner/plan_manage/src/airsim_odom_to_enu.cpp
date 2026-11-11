#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("airsim_odom_to_enu");

  auto enu_pub = node->create_publisher<nav_msgs::msg::Odometry>(
      "/grid_map/odom_enu", 20);

  auto ned_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/airsim_node/PX4/odom_local_ned", 20,
      [enu_pub](nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        const auto &p = msg->pose.pose.position;
        const auto &v = msg->twist.twist.linear;
        const auto &o = msg->pose.pose.orientation;

        Eigen::Vector3d pn(p.x, p.y, p.z);
        Eigen::Vector3d vn(v.x, v.y, v.z);
        Eigen::Quaterniond qn(o.w, o.x, o.y, o.z);

        if (!pn.allFinite() || !vn.allFinite() ||
            !qn.coeffs().allFinite() || qn.norm() < 1e-6)
          return;

        // 世界坐标：NED -> ENU
        Eigen::Matrix3d S;
        S << 0, 1, 0,
             1, 0, 0,
             0, 0,-1;

        // 机体坐标：FLU -> FRD，逆变换也是该矩阵
        Eigen::Matrix3d D;
        D << 1, 0, 0,
             0,-1, 0,
             0, 0,-1;

        Eigen::Vector3d pe = S * pn;
        Eigen::Vector3d ve = S * vn;

        // R_ENU_FLU = R_ENU_NED * R_NED_FRD * R_FRD_FLU
        Eigen::Quaterniond qe(
            S * qn.normalized().toRotationMatrix() * D);
        qe.normalize();

        nav_msgs::msg::Odometry out;
        out.header = msg->header;  // 保留传感器时间戳
        out.header.frame_id = "world";
        out.child_frame_id = "base_link";

        out.pose.pose.position.x = pe.x();
        out.pose.pose.position.y = pe.y();
        out.pose.pose.position.z = pe.z();

        out.pose.pose.orientation.w = qe.w();
        out.pose.pose.orientation.x = qe.x();
        out.pose.pose.orientation.y = qe.y();
        out.pose.pose.orientation.z = qe.z();

        // 本工程的 AirSim 原始线速度为世界 NED。
        // EGO 的 odometryCallback 需要世界 ENU 线速度。
        out.twist.twist.linear.x = ve.x();
        out.twist.twist.linear.y = ve.y();
        out.twist.twist.linear.z = ve.z();

        out.twist.twist.angular.x = msg->twist.twist.angular.x;
        out.twist.twist.angular.y = -msg->twist.twist.angular.y;
        out.twist.twist.angular.z = -msg->twist.twist.angular.z;

        enu_pub->publish(out);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
