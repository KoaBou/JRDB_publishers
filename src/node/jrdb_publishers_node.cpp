#include <chrono>

#include "jrdb_publishers/jrdb_publishers_node.hpp"

using namespace cv;
using namespace std::chrono_literals;

JRDBPublishersNode::JRDBPublishersNode()
: Node("jrdb_publisher_node"), file_index_(0)
{
    publisher_point_cloud_upper_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("jrdb/point_cloud/upper", 10);
    publisher_point_cloud_lower_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("jrdb/point_cloud/lower", 10);
    publisher_image_left_ = this->create_publisher<sensor_msgs::msg::Image>("jrdb/image/left", 10);
    publisher_image_mid_left_ = this->create_publisher<sensor_msgs::msg::Image>("jrdb/image/midleft", 10);
    publisher_image_mid_ = this->create_publisher<sensor_msgs::msg::Image>("jrdb/image/mid", 10);
    publisher_image_mid_right_ = this->create_publisher<sensor_msgs::msg::Image>("jrdb/image/midright", 10);
    publisher_image_right_ = this->create_publisher<sensor_msgs::msg::Image>("jrdb/image/right", 10);

    this->declare_parameter("dataFolder", "bytes-cafe-2019-02-07_0");

    init_file_path();

    create_publishers_data_file_names();

    timer_ = create_wall_timer(
        100ms, std::bind(&JRDBPublishersNode::on_timer_callback, this)
    );
}

void JRDBPublishersNode::on_timer_callback()
{
    sensor_msgs::msg::PointCloud2 point_cloud2_lower_msg;
    std::string pcl_pat_lower = path_point_cloud_lower_ + file_names_point_cloud_lower_[file_index_];
    convert_pcl_to_pointcloud2(point_cloud2_lower_msg, pcl_pat_lower);

    sensor_msgs::msg::PointCloud2 point_cloud2_upper_msg;
    std::string pcl_pat_upper = path_point_cloud_upper_ + file_names_point_cloud_upper_[file_index_];
    convert_pcl_to_pointcloud2(point_cloud2_upper_msg, pcl_pat_upper);

    auto image_message_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_left = path_image_left_ + file_names_image_left_[file_index_];
    convert_image_to_msg(*image_message_left, img_pat_left);

    auto image_message_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_right = path_image_right_ + file_names_image_right_[file_index_];
    convert_image_to_msg(*image_message_right, img_pat_right);

    auto image_message_mid_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_mid_right = path_image_mid_right_ + file_names_image_mid_right_[file_index_];
    convert_image_to_msg(*image_message_mid_right, img_pat_mid_right);

    auto image_message_mid_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_mid_left = path_image_mid_left_ + file_names_image_mid_left_[file_index_];
    convert_image_to_msg(*image_message_mid_left, img_pat_mid_left);

    auto image_message_mid = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_mid = path_image_mid_ + file_names_image_mid_[file_index_];
    convert_image_to_msg(*image_message_mid, img_pat_mid);

    publisher_point_cloud_upper_->publish(point_cloud2_upper_msg);
    publisher_point_cloud_lower_->publish(point_cloud2_lower_msg);
    publisher_image_left_->publish(std::move(image_message_left));
    publisher_image_right_->publish(std::move(image_message_right));
    publisher_image_mid_left_->publish(std::move(image_message_mid_left));
    publisher_image_mid_right_->publish(std::move(image_message_mid_right));
    publisher_image_mid_->publish(std::move(image_message_mid));

    RCLCPP_INFO(this->get_logger(), "Published %ith msg", file_index_);


    file_index_++;
}

void JRDBPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg, const std::string path){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud)==-1)
    {
      RCLCPP_INFO(this->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
      exit(EXIT_FAILURE);
    }

    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
}

void JRDBPublishersNode::init_file_path()
{
    std::string dataFolder = this->get_parameter("dataFolder").as_string();

    path_point_cloud_upper_ = "data/train_dataset_with_activity/pointclouds/upper_velodyne/" + dataFolder + "/";
    path_point_cloud_lower_ = "data/train_dataset_with_activity/pointclouds/lower_velodyne/" + dataFolder + "/";
    path_image_left_ = "/home/ngin/pcl_detect_ws/data/train_dataset_with_activity/images/image_0/" + dataFolder + "/";
    path_image_mid_left_ = "/home/ngin/pcl_detect_ws/data/train_dataset_with_activity/images/image_2/" + dataFolder + "/";
    path_image_mid_ = "/home/ngin/pcl_detect_ws/data/train_dataset_with_activity/images/image_4/" + dataFolder + "/";
    path_image_mid_right_ = "/home/ngin/pcl_detect_ws/data/train_dataset_with_activity/images/image_6/" + dataFolder + "/";
    path_image_right_ = "/home/ngin/pcl_detect_ws/data/train_dataset_with_activity/images/image_8/" + dataFolder + "/";
}

std::string JRDBPublishersNode::get_path(JRDBPublishersNode::PublisherType publisher_type)
{
  RCLCPP_INFO(this->get_logger(), "get_path: '%i'", publisher_type);
  std::string path;
  if (publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_UPPER){
    path = path_point_cloud_upper_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_LOWER){
    path = path_point_cloud_lower_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_LEFT){
    path = path_image_left_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_RIGHT){
    path = path_image_right_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID){
    path = path_image_mid_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID_LEFT){
    path = path_image_mid_left_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID_RIGHT){
    path = path_image_mid_right_;
  }
  return path;
}

std::vector<std::string> JRDBPublishersNode::get_filenames(PublisherType publisher_type)
{
  if (publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_UPPER){
     return file_names_point_cloud_upper_;
  }
  else if(publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_LOWER){
    return file_names_point_cloud_lower_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_LEFT){
     return file_names_image_left_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_RIGHT){
     return file_names_image_right_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID){
     return file_names_image_mid_;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID_RIGHT){
    return file_names_image_mid_right_;
  }

  return file_names_image_mid_left_;
}

void JRDBPublishersNode::set_filenames(PublisherType publisher_type, std::vector<std::string> file_names)
{
  if (publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_UPPER){
      file_names_point_cloud_upper_= file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::POINT_CLOUD_LOWER){
      file_names_point_cloud_lower_= file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_LEFT){
      file_names_image_left_= file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_RIGHT){
      file_names_image_right_= file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID_LEFT){
      file_names_image_mid_left_= file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID_RIGHT){
      file_names_image_mid_right_ = file_names;
  }else if(publisher_type == JRDBPublishersNode::PublisherType::IMAGE_MID){
      file_names_image_mid_ = file_names;
  }
}

void JRDBPublishersNode::create_publishers_data_file_names()
{
  for ( int type_index = 0; type_index != 7; type_index++ )
  {
    JRDBPublishersNode::PublisherType type = static_cast<JRDBPublishersNode::PublisherType>(type_index);
    std::vector<std::string> file_names = get_filenames(type);

   try
   {
      for (const auto & entry : std::filesystem::directory_iterator(get_path(type))){
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename());
        }
      }

      //Order lidar file names
      std::sort(file_names.begin(), file_names.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs  < rhs ;
            });
      set_filenames(type, file_names);
    }catch (const std::filesystem::filesystem_error & e)
    {
        RCLCPP_ERROR(this->get_logger(), "File path not found.");
    }
  }
}

void JRDBPublishersNode::convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path  )
{
  Mat frame;
  frame = imread(path);
  if (frame.empty())                      // Check for invalid input
  {
    RCLCPP_ERROR(this->get_logger(), "Image does not exist. Check your files path!");
    rclcpp::shutdown();
  }

  msg.height = frame.rows;
  msg.width = frame.cols;
  std::string type = mat_type2encoding(frame.type());
  msg.encoding = type;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->now();
}

std::string JRDBPublishersNode::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}
