# f9p_bringup_ros

ROS bringup package for a ZED camera, ZED IMU, two u-blox M9N receivers, one u-blox F9P receiver, NTRIP RTCM input, RViz visualization, and rosbag recording.

The ROS package name is currently `zed_cpu`.

## Features

- Publishes ZED left and right camera images.
- Publishes raw ZED IMU data on `/imu`.
- Resamples IMU data to `/imu_data_interpolated`.
- Starts three u-blox receivers:
  - `/m9n0/ublox_m9n0` on `/dev/ttyM9N0`
  - `/m9n1/ublox_m9n1` on `/dev/ttyM9N1`
  - `/f9p/ublox_f9p` on `/dev/ttyF9P`
- Starts an NTRIP client that publishes RTCM corrections on `/rtcm`.
- Provides RViz and rqt_plot launch support.
- Provides a rosbag recorder for GPS, IMU, and compressed ZED image topics.

## Requirements

- Linux with ROS Noetic
- Catkin workspace
- ZED 2, ZED 2i, or ZED Mini
- u-blox M9N and F9P receivers
- OpenCV
- HIDAPI
- LIBUSB
- ROS packages:
  - `cv_bridge`
  - `image_transport`
  - `roscpp`
  - `sensor_msgs`
  - `std_msgs`
  - `visualization_msgs`
  - `ublox_gps`
  - `ntrip_client`
  - `rtcm_msgs`
  - `rviz`
  - `rqt_plot`

## Build

Clone this repository into a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/viniciusmenesessouza/f9p_bringup_ros.git
```

Install system dependencies:

```bash
sudo apt update
sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev libopencv-dev libopencv-viz-dev
```

Install ROS dependencies:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
catkin_make
source devel/setup.bash
```

## ZED udev Rule

The ZED camera exposes its sensors as USB HID devices. Install the included udev rule before running the camera nodes:

```bash
cd ~/catkin_ws/src/f9p_bringup_ros/udev
bash install_udev_rule.sh
```

Reconnect the camera after installing the rule.

## Device Names

The launch files expect these serial device paths:

```text
/dev/ttyM9N0
/dev/ttyM9N1
/dev/ttyF9P
```

Create stable udev rules or symlinks for the receivers before launching the full stack.

## NTRIP configuration

For each .launch edit the NTRIP configurations to the ones of your given host:
```text
<param name="host" value="YOUR_HOST_ADDRESS"/> <!-- e.g., "ntrip.example.com" -->
<param name="port" value="YOUR_HOST_PORT"/> <!-- e.g., 2101 -->
<param name="mountpoint" value="YOUR_HOST_MOUNTPOINT"/> <!-- e.g., "RTCM3" -->
<param name="user" value="YOUR_USER"/> <!-- e.g., "myusername" -->
<param name="password" value="YOUR_PASSWORD"/> <!-- e.g., "mypassword" -->
```

## Launch

Start the ZED camera and IMU pipeline:

```bash
roslaunch zed_cpu zed.launch
```

Start the ZED, IMU, GPS receivers, and NTRIP client:

```bash
roslaunch zed_cpu zed_gps.launch
```

Start the full visualization setup with RViz, TF publishers, IMU marker, and rqt plots:

```bash
roslaunch zed_cpu zed_rviz.launch
```

Before using the NTRIP launch files, review the server, mountpoint, username, and password parameters in the launch file.

## Recording

Record the configured GPS, IMU, and compressed ZED image topics:

```bash
roslaunch zed_cpu zed_record.launch
```

By default, bags are written to `~/bags` with filenames like:

```text
HHMMSS_DDMMYY_VIENA.bag
```

Override the output directory or tag:

```bash
roslaunch zed_cpu zed_record.launch bag_dir:=/path/to/bags tag:=FIELD_RUN
```

The recorder can also include TF topics:

```bash
roslaunch zed_cpu zed_record.launch record_tf:=true
```

## Main Topics

- `/imu`
- `/imu_data_interpolated`
- `/zed_node/rgb/left_image/compressed`
- `/zed_node/rgb/right_image/compressed`
- `/f9p/ublox_f9p/fix`
- `/f9p/ublox_f9p/fix_velocity`
- `/f9p/ublox_f9p/navpvt`
- `/f9p/ublox_f9p/navsat`
- `/f9p/ublox_f9p/navstatus`
- `/f9p/ublox_f9p/rxmrtcm`
- `/m9n0/ublox_m9n0/fix`
- `/m9n1/ublox_m9n1/fix`

## Notes

- Camera resolution and FPS are configured in the launch files with `camera_resolution` and `camera_fps`.
- The IMU resampler publishes at `300.0 Hz` by default.
- The GPS receivers are configured at `5 Hz` by default.
- The F9P launch configuration sets `dgnss_mode` to `RTK FIXED`.
