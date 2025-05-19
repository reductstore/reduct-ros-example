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
ls /dev/video*
# You should see /dev/video0 (and possibly /dev/video1, etc. if multiple video capture interfaces are connected)
```

Run the USB camera (e.g. /dev/video0 at 1280x720):

```bash
bash
````
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[1280,720]" -p video_device:="/dev/video0"
```

## Ressources

- [ReductStore](https://www.reduct.store)
- [ROS2 Python Tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)
- [ROS2 USB Cam Package](https://index.ros.org/p/usb_cam/)
