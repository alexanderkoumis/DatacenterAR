#include "NodeSesh.hpp"

#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void GetPublishedTopics(std::vector<std::string>& topics) {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (auto& topic : master_topics) {
    topics.push_back(topic.name);
    std::cout << "Topic name: " << topic.name << std::endl;
  }
}

bool TopicExists(const std::string& topic_new) {
  std::vector<std::string> topics;
  GetPublishedTopics(topics);

  for (auto& topic_existing : topics) {
    if (topic_existing == topic_new) {
      return true;
    }
  }
  return false;
}

void NodeSesh::PoseCallback(const geometry_msgs::PoseStamped& msg) {
  
  std::ostringstream oss;
  oss << msg.pose.position.x << ","
      << msg.pose.position.y << ","
      << msg.pose.position.z << ","
      << msg.pose.orientation.x << ","
      << msg.pose.orientation.y << ","
      << msg.pose.orientation.z << ","
      << msg.pose.orientation.w;

  NodeSendPoseArgs* args = new NodeSendPoseArgs {callback_, oss.str().c_str()};

  asyncSendPose_.data = (void*) args;

  uv_async_send(&asyncSendPose_);
}

NodeSesh::NodeSesh() : cam_info_(new sensor_msgs::CameraInfo) {
  int argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "image_publisher");

  ros::NodeHandle nh_i;
  ros::NodeHandle nh_o;

  nh_i.setCallbackQueue(&cb_i_);
  nh_o.setCallbackQueue(&cb_o_);

  image_transport::ImageTransport it(nh_i);

  std::cout << "Topic exists: " << TopicExists("/nodejs_link/image") << std::endl;

  pub_ = it.advertiseCamera("/nodejs_link/image", 10);
  sub_ = nh_o.subscribe("/lsd_slam/pose", 100, &NodeSesh::PoseCallback, this);

  ros::AsyncSpinner spinner_i(3, &cb_i_);
  ros::AsyncSpinner spinner_o(2, &cb_o_);

  spinner_i.start();
  spinner_o.start();
}

NodeSesh::~NodeSesh() {
  ros::waitForShutdown();
}

void NodeSesh::FeedPicture(const cv::Mat& image_orig, int focal_x, int focal_y, int channels) {
 
  if (image_orig.empty()) {
    std::cout << "Error: image empty" << std::endl;
    return;
  }

  if (image_orig.rows <= 0 || image_orig.cols <= 0) {
    std::cout << "Error: image size: " << image_orig.size() << std::endl;
    return;
  }

  const int cols_orig = image_orig.cols;
  const int rows_orig = image_orig.rows;

  const bool divisible_by_16_rows = cols_orig % 16;
  const bool divisible_by_16_cols = rows_orig % 16;
  const bool needs_to_be_resized = !divisible_by_16_rows || !divisible_by_16_cols;


  cv::Mat image = image_orig.clone();

  if (needs_to_be_resized) {

    const int cols_new = cols_orig - (cols_orig % 16) + 16;
    const int rows_new = rows_orig - (rows_orig % 16) + 16;

    focal_x *= (cols_new / cols_orig); 
    focal_y *= (rows_new / rows_orig); 

    cv::resize(image, image, cv::Size(cols_new, rows_new));

  }

  std::string img_mode;
  switch (channels) {
    case 1: img_mode = "mono8"; break;
    case 3: img_mode = "rgb8"; break;
    case 4: img_mode = "rgba8"; break;
    default: img_mode = "rgb8"; break;
  }

  auto header = std_msgs::Header();
  header.stamp = ros::Time::now();
  cv_bridge::CvImage img_ros(header, img_mode, image);
  sensor_msgs::ImagePtr img_msg = img_ros.toImageMsg();

  std::call_once(once_info_, [&] () {

    const double fx = focal_x;
    const double fy = focal_y;
    const double cx = image.cols/2;
    const double cy = image.rows/2;

    cam_info_->K = {fx,   0.0f, cx,
                    0.0f, fy,   cy,
                    0.0f, 0.0f, 1.0f};

    cam_info_->R = {1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f};

    cam_info_->P = {fx,   0.0f, cx,   0.0,
                    0.0f, fy,   cy,   0.0,
                    0.0f, 0.0f, 1.0f, 0.0};

    cam_info_->width = image.cols;
    cam_info_->height = image.rows;

  });

  pub_.publish(img_msg, cam_info_);
  cb_o_.callAvailable(ros::WallDuration());
}
