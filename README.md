# reduct-ros-example
This application demonstrates a simple setup for using ROS 2 with ReductStore.

## Python environment

To use Python environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

You need to add COLCON_IGNORE to the .venv directory to avoid colcon trying to build it.

```bash
touch .venv/COLCON_IGNORE
```

## Build with colcon

Build the ROS package:

```bash
colcon build --packages-select reduct_camera
```

Source the ROS package:

```bash
source install/local_setup.bash
```

## Run ReductStore in a Docker container

Run the ReductStore container:

```bash
docker run -d -p 8383:8383 -v ${PWD}/data:/data reduct/store:latest
```

## Run the ROS package


Run the package with the default parameters:

```bash
ros2 run reduct_camera capture_and_store
```

## Test the application with a USB camera

Launch the webcam:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./usb_cam_config.yaml
```

## Ressources

- [ReductStore](https://www.reduct.store)
- [ROS2 Python Tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)
- [ROS2 USB Cam Package](https://index.ros.org/p/usb_cam/)
