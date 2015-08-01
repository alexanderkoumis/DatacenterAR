#include "NodeSesh.hpp"

#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static cv::Mat ImageRsz16(const cv::Mat& image_in) {
  cv::Mat image_out;
  const int rows = image_in.rows;
  const int cols = image_in.cols;
  const int rows_new = rows - (rows % 16) + 16;
  const int cols_new = cols - (cols % 16) + 16;
  cv::resize(image_in, image_out, cv::Size(cols_new, rows_new));
  return image_out;
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

NodeSesh::NodeSesh() : cam_info_(new sensor_msgs::CameraInfo()) {
  int argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "image_publisher");

  ros::NodeHandle nh_i;
  ros::NodeHandle nh_o;

  nh_i.setCallbackQueue(&cb_i_);
  nh_o.setCallbackQueue(&cb_o_);

  image_transport::ImageTransport it(nh_i);

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

void NodeSesh::FeedPicture(const cv::Mat& image_orig, int channels) {

  if (image_orig.empty()) {
    std::cout << "Error: image empty" << std::endl;
    return;
  }

  cv::Mat image;

  const bool divisible_by_16_rows = image_orig.rows % 16;
  const bool divisible_by_16_cols = image_orig.rows % 16;

  image = (!divisible_by_16_cols || !divisible_by_16_rows) ? ImageRsz16(image_orig) : image_orig;

  std::string img_mode;

  switch (channels) {
    case 1: img_mode = "mono8"; break;
    case 3: img_mode = "rgb8"; break;
    case 4: img_mode = "rgba8"; break;
    default: img_mode = "rgb8"; break;
  }

  cv_bridge::CvImage header(std_msgs::Header(), img_mode, image);
  sensor_msgs::ImagePtr msg = header.toImageMsg();

  std::call_once(once_info_, [&] () {
    const double fx = 525.0;
    const double fy = 525.0;
    const double cx = image.cols/2;
    const double cy = image.rows/2;

    cam_info_->K = {fx,   0.0f, cx,
                    0.0f, fy,   cy,
                    0.0f, 0.0f, 1.0f};

    cam_info_->P = {1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0};

    cam_info_->width = image.cols;
    cam_info_->height = image.rows;
  });

  pub_.publish(msg, cam_info_);
  cb_o_.callAvailable(ros::WallDuration());
}
