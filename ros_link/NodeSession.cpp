#include "NodeSession.hpp"

#include <node_buffer.h>
#include <uv.h>

#include <opencv2/highgui/highgui.hpp>

struct NodeFeedPictureArgs { 
  NodeSesh* obj_;
  void* picture_;
  int channels_;
};

PersistentFunc NodeSession::constructor;

void NodeBindFeedPicture(uv_async_s* handle) {
  NodeFeedPictureArgs* args = (NodeFeedPictureArgs*)(handle->data);
  cv::Mat picture(*((cv::Mat*)args->picture_));
  args->obj_->FeedPicture(picture, args->channels_);
  delete args;
}

void NodeBindSendPose(uv_async_s* handle) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);

  NodeSendPoseArgs* args = (NodeSendPoseArgs*)(handle->data);
  v8::Handle<v8::String> pose_str = v8::String::NewFromUtf8(isolate, args->pose_cstr_);
  PersistentFunc callback_pers(isolate, args->callback_);
  v8::Local<v8::Function> callback = v8::Local<v8::Function>::New(isolate, callback_pers);
  
  if (strlen(args->pose_cstr_) > 1) {
    const int argc = 1;
    v8::Handle<v8::Value> argv [argc] = {pose_str};
    callback.As<v8::Function>()->Call(callback, argc, argv);    
  }

  delete args;
}

NodeSession::~NodeSession() {}

void NodeSession::Init(v8::Handle<v8::Object> exports) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();

  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = v8::FunctionTemplate::New(isolate, New);
  tpl->SetClassName(v8::String::NewFromUtf8(isolate, ""));
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  // Prototypes
  NODE_SET_PROTOTYPE_METHOD(tpl, "feedPictureFile", BindFeedPictureFile);
  NODE_SET_PROTOTYPE_METHOD(tpl, "feedPictureBlob", BindFeedPictureBlob);

  constructor.Reset(isolate, tpl->GetFunction());
  exports->Set(v8::String::NewFromUtf8(isolate, "NodeSession"), tpl->GetFunction());
}

void NodeSession::New(const v8::FunctionCallbackInfo<v8::Value>& args) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);
  
  if (args.IsConstructCall()) {

    NodeSession* node_session = new NodeSession();
    node_session->Wrap(args.This());
    node_session->Ref();
    node_session->node_sesh_ = new NodeSesh();

    // Threading
    node_session->node_sesh_->loop_ = uv_loop_new();

    uv_async_init(node_session->node_sesh_->loop_,
                  &node_session->node_sesh_->asyncBindFeedPicture_,
                  NodeBindFeedPicture);

    uv_async_init(uv_default_loop(),
                  &node_session->node_sesh_->asyncSendPose_,
                  NodeBindSendPose);

    uv_thread_create(&node_session->node_sesh_->thread_, [](void* n) {
      NodeSesh* sesh = (NodeSesh*) n;
      uv_run(sesh->loop_, UV_RUN_DEFAULT);
    }, node_session->node_sesh_);

    args.GetReturnValue().Set(args.This());
  }
  else {
    const int argc = 1;
    v8::Local<v8::Value> callback = args[0];
    v8::Local<v8::Value> argv[argc] = {callback};
    v8::Local<v8::Function> cons = v8::Local<v8::Function>::New(isolate, constructor);
    args.GetReturnValue().Set(cons->NewInstance(argc, argv));
  }
}

void NodeSession::NewInstance(const v8::FunctionCallbackInfo<v8::Value>& args) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);

  const int argc = 1;
  v8::Local<v8::Value> callback = args[0];
  v8::Handle<v8::Value> argv[argc] = {callback};
  v8::Local<v8::Function> cons = v8::Local<v8::Function>::New(isolate, constructor);
  args.GetReturnValue().Set(cons->NewInstance(argc, argv));
}

void NodeSession::BindFeedPictureBlob(const v8::FunctionCallbackInfo<v8::Value>& args) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);

  std::vector<unsigned char> v_picture;

  char* ptr_pic = node::Buffer::Data(args[0]);
  int cols = args[1]->NumberValue();
  int rows = args[2]->NumberValue();
  int channels = args[3]->NumberValue();

  int img_mode = 0;

  switch (channels) {
    case 1: img_mode = CV_8UC1; break;
    case 3: img_mode = CV_8UC3; break;
    case 4: img_mode = CV_8UC4; break;
  }

  cv::Mat picture = cv::Mat(cv::Size2i(cols, rows), img_mode, ptr_pic);

  NodeSession* node_session = ObjectWrap::Unwrap<NodeSession>(args.This());

  NodeFeedPictureArgs* call_args = new NodeFeedPictureArgs {
      node_session->node_sesh_,
      (void*) new cv::Mat(picture),
      channels
  };
  
  v8::Handle<v8::Function> callback = v8::Handle<v8::Function>::Cast(args[4]);
  node_session->node_sesh_->asyncBindFeedPicture_.data = (void*) call_args;
  node_session->node_sesh_->callback_ = PersistentFunc(isolate, callback);

  uv_async_send(&node_session->node_sesh_->asyncBindFeedPicture_);

  args.GetReturnValue().Set(Null(isolate));
}

void NodeSession::BindFeedPictureFile(const v8::FunctionCallbackInfo<v8::Value>& args) {
  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);

  std::vector<unsigned char> v_picture;

  char* ptr_pic = node::Buffer::Data(args[0]);
  const size_t len_pic = node::Buffer::Length(args[0]);

  int channels = args[1]->NumberValue();

  const unsigned char* data_pic = (const unsigned char *) ptr_pic;

  v_picture.assign(data_pic, data_pic + len_pic);

  cv::Mat picture(cv::imdecode(v_picture, -1));

  NodeSession* node_session = ObjectWrap::Unwrap<NodeSession>(args.This());

  NodeFeedPictureArgs* call_args = new NodeFeedPictureArgs {
      node_session->node_sesh_,
      (void*) new cv::Mat(picture),
      channels
  };
  
  v8::Handle<v8::Function> callback = v8::Handle<v8::Function>::Cast(args[2]);
  node_session->node_sesh_->asyncBindFeedPicture_.data = (void*) call_args;
  node_session->node_sesh_->callback_ = PersistentFunc(isolate, callback);

  uv_async_send(&node_session->node_sesh_->asyncBindFeedPicture_);

  args.GetReturnValue().Set(Null(isolate));
}
