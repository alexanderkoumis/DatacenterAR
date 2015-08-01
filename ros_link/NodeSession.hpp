#ifndef NODE_SESSION_HPP
#define NODE_SESSION_HPP

#include <node_object_wrap.h>
#include <opencv2/core/core.hpp>

#include "NodeSesh.hpp"

class NodeSession : public node::ObjectWrap {
public:
  ~NodeSession();
  static void Init(v8::Handle<v8::Object> exports);
  static void NewInstance(const v8::FunctionCallbackInfo<v8::Value>& args);

private:
  void CallSuperInit();

  static void New(const v8::FunctionCallbackInfo<v8::Value>& args);
  static PersistentFunc constructor;

  static void BindFeedPictureBlob(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void BindFeedPictureFile(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void BindSendPose(const v8::FunctionCallbackInfo<v8::Value>& args);

  NodeSesh* node_sesh_;
};

#endif