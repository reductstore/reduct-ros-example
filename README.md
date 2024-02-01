# reduct-ros-example
This application demonstrates a simple setup for using ROS with ReductStore.

## Build

```bash
colcon build --packages-select reduct_camera
```

```bash
source install/local_setup.bash
```

## Run

```bash
ros2 run reduct_camera capture_and_store
```

## Demo

Launch the webcam:

```bash
ros2 run usb_cam usb_cam_node_exe
```