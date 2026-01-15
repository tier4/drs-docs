# LiDAR-LiDAR Calibration

This page describes the procedure for calibrating the relative pose between multiple LiDARs using a mapping-based approach.

Tool reference: [mapping_based_calibrator.md](https://github.com/tier4/CalibrationTools/blob/feat/drs/docs/tutorials/mapping_based_calibrator.md)

> [!WARNING]
> The alignment calculation after playback can take **30 minutes or more**. Ensure your environment is stable during this time.

---

## 1. Data Collection (ECU Side)

Calibrating multiple LiDARs requires high-quality point cloud data from an environment with varied features.

> [!NOTE]
> Depending on the driver used, the topic name may be `seyond_packets` instead of `nebula_packets`.

1.  **Preparation**: Drive the vehicle to an open area with some structures (e.g., walls, pillars, or parked vehicles) for better feature matching.
2.  **Record Data**: Drive in a **figure-eight or oval trajectory** to ensure all LiDARs capture overlapping features from different angles.
3.  **Execute Command**:
    ```bash
    # SSH into an ECU (ECU0 or ECU1)
    # Record all LiDAR packet topics to an MCAP file
    ros2 bag record -s mcap \
      /sensing/lidar/front/nebula_packets \
      /sensing/lidar/right/nebula_packets \
      /sensing/lidar/rear/nebula_packets \
      /sensing/lidar/left/nebula_packets \
      -o <BAG_NAME>
    ```
    ![LiDAR Recording](images/image-2025-09-03-13-45-14.png)

---

## 2. Calibration Procedure (PC Side)

Process the recorded data on the calibration PC to compute the extrinsic parameters.

### Step 1: Launch Services
You need to run the decoder and the calibration manager in separate environments.

1.  **Terminal 1 (Runtime Container)**: Launch the LiDAR packet decoder.
    ```bash
    ros2 launch drs_launch drs_offline.launch.xml publish_tf:=false
    ```

    > [!NOTE]
    > If you are using the Seyond LiDAR driver, add the argument `lidar_driver_type:=seyond` to the launch command.
2.  **Terminal 2 (Calibration Container)**: Start the calibration manager.
    ```bash
    ros2 run sensor_calibration_manager sensor_calibration_manager
    ```

### Step 2: Configure the Tool
1.  **Select Calibrator**: Choose `mapping_based_lidar_lidar_calibrator`.
2.  **Initial Values**: Set the `imu_to_front_*` values and other initial TFs based on the vehicle design or CAD values.

### Step 3: Play the Rosbag
1.  **Execute Playback**:
    ```bash
    ros2 bag play <BAG_PATH> --clock 100 -r 0.1
    ```
2.  **Wait**: The playback speed is set to `0.1x` to ensure the tool has enough time to process the packets. The tool automatically controls pausing and resuming.

---

## 3. Finalization

1.  **Stop Mapping**: Once playback finishes, call the stop service to trigger the final alignment calculation.
    ```bash
    ros2 service call /stop_mapping std_srvs/srv/Empty
    ```
2.  **Wait for Alignment**: Monitor the terminal/UI for the "Alignment finished" status (this may take 30+ minutes).

---

**Next Step**: [Result integration and Application](result_integration.md)
