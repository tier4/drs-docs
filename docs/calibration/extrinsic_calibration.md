# Camera-LiDAR Extrinsic Calibration

This page describes the procedure for calibrating the relative pose between the camera and the LiDAR using an AprilTag board.

Tool reference: [tag_based_pnp_calibrator.md](https://github.com/tier4/CalibrationTools/blob/feat/drs/docs/tutorials/tag_based_pnp_calibrator.md)

> [!NOTE]
> This procedure assumes a specific ID and orientation for the AprilTag. Mount and orient the board as shown below:  
> ![Extrinsic Calibration Target](images/extrinsic_calib_target.png)

Next step: [Camera-LiDAR Intrinsic Calibration](camera_lidar_intrinsic_calibration.md)

---

## 1. Preparation

### ECU Side: Start Sensor Streams
Calibration requires sensor data without coordinate transformations (TF).

```bash
# SSH into the ECU0 and ECU1 that target sensors are connected
# example ECU0: ssh nvidia@192.168.20.1 
sudo systemctl stop drs-sensor.service

# Start DRS services without TF
ros2 launch drs_launch drs.launch.xml publish_tf:=false param_root_dir:=/opt/drs/config/params
```

### PC Side: Launch Decoder & Tool
1.  **Launch Decoder (Runtime Container or Local Build Environment)**: Run the offline decoder to process sensor packets.
    ```bash
    ros2 launch drs_launch drs_offline.launch.xml publish_tf:=false
    ```

    > [!NOTE]
    > If you are using the Seyond LiDAR driver, add the argument `lidar_driver_type:=seyond` to the launch command.

2.  **Launch Tool (Calibration Container or Local Build Environment)**: Start the sensor calibration manager.
    ```bash
    ros2 run sensor_calibration_manager sensor_calibration_manager
    ```

---

## 2. Calibration Procedure

### Step 1: Initial Configuration
1.  **First Dialog**:
    - **Project**: `drs`
    - **Calibrator**: `tag_based_pnp_calibrator`
    - Click **Continue**.  
    ![First Dialog](images/image-20241120-124937.png)
2.  **Second Dialog**:
    - **Camera Name**: Select the target camera (e.g., `camera0`). The tool will automatically select the corresponding LiDAR.
    - Click **Launch**.  
    ![Second Dialog](images/extrinsic_second_dialog.png)

### Step 2: UI Setup and start calibration
Adjust the settings in the tool and RViz:

- **Calibration Tool/Image view**:
    - **TF source**: `Current /tf`
    - **Marker units**: `Pixels`  
    ![Image View](images/image-20241121-112246.png)
- **Calibration Tool/RViz2**:
    - **Fixed Frame**: Set to the appropriate LiDAR frame (e.g., `lidar_front`).  
    ![RViz2](images/image-20241121-112752.png)
- **Calibration Tool/sensor_calibration_manager**:
    - Click **Calibrate**.  
    ![Calibrate](images/image-20241121-113015.png)


### Step 3: Data Collection
1.  Move the AprilTag board slowly within the sensor field of view.
2.  Monitor the `crossvalidation_reprojection_error` in the tool.
3.  **Completion Criteria**: When the number of detected pairs exceeds **14**, click **Save calibration**.
4.  **Rename Result**: Rename the generated file to `camera<N>_calibration_results.yaml`.

---

## 3. Troubleshooting

### No images in image_view
If the "delay" value in the UI is too high, the images may not display. This is often caused by time synchronization issues between the ECU and PC.

**Fix**: Relax the delay tolerance in the following file:
`common/tier4_calibration_views/tier4_calibration_views/image_view_ros_interface.py`

```python
# Change the default value from 0.06 to 1.06
self.declare_parameter("delay_tolerance", 1.06)
```

### AprilTag Not Detected
- Ensure the board is well-lit and not tilted at an extreme angle.
- Verify the LiDAR detects the board surface as a flat plane.
