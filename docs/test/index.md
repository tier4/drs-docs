# DRS test

- [\[Optional\] Preparation](#optional-preparation)
- [Topic rate](#topic-rate)
- [Recording test](#recording-test)
- [PTP synchronization](#ptp-synchronization)
    - [GNSS-ECU synchronization](#gnss-ecu-synchronization)
    - [ECU-ECU synchronization](#ecu-ecu-synchronization)
    - [ECU-sensors synchronization](#ecu-sensors-synchronization)
- [LiDAR/camera timestamp consistency](#lidarcamera-timestamp-consistency)
- [LiDAR-LiDAR calibration accuracy](#lidar-lidar-calibration-accuracy)
- [LiDAR-camera calibration accuracy](#lidar-camera-calibration-accuracy)
- [Whole operation check](#whole-operation-check)


# [Optional] Preparation

- Enable internet access temporaly 
  ```shell
  # temporaly enable internet access
  route add default gw <VPN_ROUTER_IP>
  ```

- Install utility tools to visualize the ECU status
  ```shell
  sudo apt install nmon vnstat
  ```

# Topic rate
- login to the each ECU
- Check the topic rate listed in `/opt/drs/record_topics_ecu<ECU_ID>.yaml`
  - `ros2 topic hz -w 5 <target_topic>`
  - Note: check the topic rate locally (i.e., in the ECU) to check the topic rate precisely.

# Recording test
- Check rosbags are recorded/saved on the one of the mounted SSD
  - The target SSD is switched every one hour. Ex. Between 10 and 11 o'clock, SSD#1 is used.
  ```shell
  nmon
  ```

- Check data saved in the other SSD is transferred to the NAS
  ```shell
  vnstat -i mgbe1 -l  # mgbe1 is connected to the NAS
  ```

# PTP synchronization
## GNSS-ECU synchronization
- login to the ECU#2
- check the log of `ptpd` to confirm PTP communication is in operation
  ```shell
  journalctl -f -u ptpd_slave@eth1.service
  ```

## ECU-ECU synchronization
- login to the ECU#1
- check the log of `ptpd` to confirm PTP communication is in operation
  ```shell
  journalctl -f -u ptpd_slave@eth0.service
  ```

## ECU-sensors synchronization
- Check timestamp of each topics
  ```shell
  ros2 topic echo --field header.stamp <target_topic>
  ```
- For LiDAR data, you need to check timestamps for `nebula_points` rather than `nebula_packets`
  - connect a PC to the ROS network
  - `ros2 launch drs_launch drs_offline.launch`
  - Now, you should be able to see `nebula_points` in your PC

# LiDAR/camera timestamp consistency
- login to the each ECU
- take a rosbag containing all LiDARs (`nebula_packets`) and cameras (`camera_info`)connected to the ECU for approx. 10 seconds
  - Ex. in case of ECU#1, the rosbag will contain `/sensing/lidar/(front|right)/nebula_packets` and `/sensing/camera/camera(0|1|2|3)/camera_info`
- copy the rosbag to your PC and play it with `ros2 launch drs_launch drs_offlline.launch.xml`
- Open multiple terminals and compare the timestamp of all `nebula_points` and `camera_info`
- Check all LiDARs and cameras have almost similar timestamp between +- 200ms

# LiDAR-LiDAR calibration accuracy
Sample RViz configration can be found [here](samples/lidar_lidar.rviz)

- confirm z-value of pointclouds near the ground is almost zero
  - If not the case, tune z translation value of `base_link` -> `drs_base_link`

- Put static objects, something like cones, around the vehicle and visualize pointclouds from all LiDARs using TF.
  - Confirm that the points for the objects around the vehicle overlap precisely.

# LiDAR-camera calibration accuracy
- Project LiDAR points on the image and check LiDAR points for the static objects are overlapped onto the same object in the image.
  - ref: https://github.com/tier4/data_collection_tools/tree/main/data_collection_tools#project-point
  ```shell
  python project_point.py -i CAMERA_TOPIC -p LIDAR_TOPIC --image_sync_offset OFFSET --image_timestamp_delay DELAY --show_timestamp 
  ```

- Swing a bar in front of the camera and LiDAR and confirm LiDAR points for the bar are precisely projected onto the bar in the image.

# Whole operation check
- Shutdown entier system
- Boot the system
- Push the button in the display shortly and confirm the services are restarted successfully.
- Push the button in the display approx. 10 sec  and confirm the system are shutdown successfully.
