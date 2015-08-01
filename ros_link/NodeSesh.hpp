#ifndef NODE_SESH_HPP
#define NODE_SESH_HPP

#include <iostream>
#include <memory>
#include <string>
#include <thread> // call_once
#include <memory>
#include <mutex>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <node.h>
#include <uv.h>

typedef v8::Persistent<v8::Function, v8::CopyablePersistentTraits<v8::Function>> PersistentFunc;

struct NodeSendPoseArgs {
  PersistentFunc callback_;
  const char* pose_cstr_;
};

class Session {
 public:
	virtual void FeedPicture(const cv::Mat& image, int channels) {};
};

class NodeSesh : public Session {
 public:
  NodeSesh();
  ~NodeSesh();
  virtual void FeedPicture(const cv::Mat& image, int channels);
  friend class NodeSession;
  void PoseCallback(const geometry_msgs::PoseStamped& msg);
  void SendPose(cv::Mat& quat, cv::Mat& transform);

  std::vector<unsigned char> data_vec_;
 private:
  PersistentFunc callback_;

  uv_thread_t thread_;
  uv_loop_t* loop_;
  uv_async_t asyncBindFeedPicture_;
  uv_async_t asyncSendPose_;

  ros::CallbackQueue cb_i_;
  ros::CallbackQueue cb_o_;

  ros::Subscriber sub_;
  image_transport::CameraPublisher pub_;
  sensor_msgs::CameraInfoPtr cam_info_;
  std::once_flag once_info_;

  geometry_msgs::Point coord_;
  geometry_msgs::Quaternion orientation_;

  int data_length_;

};

#endif
