#ifndef JRDB_PUBLISHERS__JRDB_PUBLISHERS_NODE_HPP_
#define JRDB_PUBLISHERS__JRDB_PUBLISHERS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdlib>

#include "jrdb_publishers/visibility.hpp"
#include "jrdb_publishers/WGS84toCartersian.hpp"

class JRDBPublishersNode : public rclcpp::Node
{
    public:
        enum class PublisherType
        {
            POINT_CLOUD_UPPER = 0,
            POINT_CLOUD_LOWER = 1,
            IMAGE_LEFT = 2,
            IMAGE_MID_LEFT = 3,
            IMAGE_MID = 4,
            IMAGE_MID_RIGHT = 5,
            IMAGE_RIGHT = 6,
        };

        KITTI_PUBLISHERS_NODE_PUBLIC JRDBPublishersNode();

        std::string get_path(PublisherType publisher_type);
        std::vector<std::string> get_filenames(PublisherType publisher_type);
        void set_filenames(PublisherType pyblisher_typem, std::vector<std::string> file_names);

    private:
        void on_timer_callback();

        void init_file_path();
        void create_publishers_data_file_names();
        std::vector<std::string> parse_file_data_into_string_array(std::string file_name, std::string delimiter);

        std::string mat_type2encoding(int mat_type);
        void convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path);

        void prepare_navsatfix_msg(std::vector<std::string> & oxts_tokenized_array, sensor_msgs::msg::NavSatFix & msg);
        void prepare_imu_msg(std::vector<std::string> & oxts_tokenized_array, sensor_msgs::msg::Imu & msg);
        void prepare_marker_array_msg(std::vector<std::string> & oxts_tokenized_array, visualization_msgs::msg::MarkerArray & msg);
        void convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg, const std::string path);

        size_t file_index_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_upper_;   // velodyne point clouds publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_lower_;   // velodyne point clouds publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_left_;     // left rectified grayscale image sequence
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_mid_left_;    // right rectified grayscale image sequence
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_right_;    // left rectified color image sequence
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_mid_right_;   // right rectified color image sequence
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_mid_;   // right rectified color image sequence
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_odometry_;            // oxts odometry publisher
        // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;                    // oxts odometry publisher
        // rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_nav_sat_fix_;      // oxts odometry publisher
        // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;  // oxts odometry publisher

        std::vector<std::string> file_names_point_cloud_upper_;
        std::vector<std::string> file_names_point_cloud_lower_;
        std::vector<std::string> file_names_image_left_;
        std::vector<std::string> file_names_image_mid_left_;
        std::vector<std::string> file_names_image_mid_;
        std::vector<std::string> file_names_image_mid_right_;
        std::vector<std::string> file_names_image_right_;
        // std::vector<std::string> file_names_oxts_;

        std::string path_point_cloud_upper_;
        std::string path_point_cloud_lower_;
        std::string path_image_left_;
        std::string path_image_mid_left_;
        std::string path_image_mid_;
        std::string path_image_mid_right_;
        std::string path_image_right_;
        // std::string path_oxts_;
};

#endif