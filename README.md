# DatacenterAR

Augmented reality datacenter simulation

## Required to build

  * [node 0.12+](https://nodejs.org/)
  * [bower](http://bower.io/)
  * [grunt](http://gruntjs.com/getting-started) (optional)
  * [ROS Indigo](http://wiki.ros.org/indigo) | [ROS Jade](http://wiki.ros.org/jade)
    * [LSD_SLAM](https://github.com/tum-vision/lsd_slam)

## Building

```bash
cd $project_dir
npm install

# with Grunt
grunt install

# without Grunt
bower install
cd ros_link
node-gyp rebuild
```

## Running

Three processes must be launched to run DatacenterAR. With Grunt (a javascript task-runner) installed, launching said processes is as simple as:

```bash
grunt
```

Killing this process (either with `SIGINT` or `SIG`) will also clean up all spawned child processes.

To launch the necessary processes without Grunt (and manually clean up spawned processes), first initialize ROS: 

```bash
roscore
```

In another terminal, launch the web app:

```bash
cd $project_dir
node js/app.js
```

When a client connects, images from their webcam, along with camera information are published by `image_transport::ImageTransport` as `/nodejs_link/image` and `/nodejs_link/camera_info`, subsequently used as input for LSD_SLAM:

```bash
rosrun lsd_slam_core live_slam image:=/nodejs_link/image camera_info:=/nodejs_link/camera_info'
```

Pose estimations are published by LSD_SLAM as `/lsd_slam/pose`, which are sent back to the client's three.js viewer to modify perspective.

## Notes

System works smoothly with modern desktop (~30-45Hz) but slows down to ~5Hz on a mobile due to getUserMedia performance constraints. Eventual implementations of [OffscreenCanvas](https://wiki.whatwg.org/wiki/OffscreenCanvas) or [toBlob()](https://code.google.com/p/chromium/issues/detail?id=67587#c101) will improve speed on mobile devices.