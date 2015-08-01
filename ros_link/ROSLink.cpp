#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>

#include <node.h>

#include "NodeSession.hpp"

std::vector<PersistentFunc> callbacks;

std::mutex mtx;

void threadMain (PersistentFunc callback) {
  mtx.lock();
  callbacks.push_back(callback);
  mtx.unlock();
}

static void update(const v8::FunctionCallbackInfo<v8::Value>& args) {

  v8::Isolate* isolate = v8::Isolate::GetCurrent();
  v8::HandleScope scope(isolate);

  for( auto i = callbacks.begin(); i != callbacks.end(); ) {

    PersistentFunc callback_per(isolate, *i);
    v8::Local<v8::Function> callback = v8::Local<v8::Function>::New(isolate, callback_per);

    v8::Handle<v8::Value> callargs[] = {};
    callback.As<v8::Function>()->Call(callback, 0, callargs);

    i = callbacks.erase(i);
  }
}

void init(v8::Handle<v8::Object> exports, v8::Handle<v8::Object> module) {
  NodeSession::Init(exports);
  NODE_SET_METHOD(exports, "Session", NodeSession::NewInstance);
  NODE_SET_METHOD(exports, "update", update);
}
NODE_MODULE(ros_link, init)