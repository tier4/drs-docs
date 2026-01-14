# Camera Intrinsic Calibration

This page describes the procedure for calibrating the internal parameters (focal length, principal point, and distortion) of the cameras.

Tool reference: [intrinsic_camera_calibrator.md](https://github.com/tier4/CalibrationTools/blob/feat/drs/docs/tutorials/intrinsic_camera_calibrator.md)

---

## 1. Preparation & Execution

Open a terminal on the calibration PC and navigate to the tool directory.

:::tip
Ensure the sensor drivers are running on the ECUs (see [Sensor Check](sensor_check.md)).
:::

**Terminal (Inside Calibration Container or Workspace):**
```bash
# Docker environment
cd /opt/drs

# Source environment (if building from source)
# cd <YOUR_WORKSPACE>/calibration_tools
# source ./install/setup.bash

# Run for C2-30 (camera0, camera4)
ros2 run intrinsic_camera_calibrator camera_calibrator \
  --config-file ./install/intrinsic_camera_calibrator/share/intrinsic_camera_calibrator/config/intrinsics_calibrator_c2_30.yaml

# Run for C2-120 (camera1, camera2, camera3, camera5, camera6, camera7)
ros2 run intrinsic_camera_calibrator camera_calibrator \
  --config-file ./install/intrinsic_camera_calibrator/share/intrinsic_camera_calibrator/config/intrinsics_calibrator_c2_120.yaml
```

---

## 2. Calibration Procedure

Follow these steps for each camera:

### Step 1: Initialization
1.  **First Dialog**:
    - **Board options**: `Chess board`
    - **Parameters Profile**: `Ceres Intrinsics Calibrator`
    - Click **Start**.
    ![First Dialog](images/1st_diag.png)
2.  **Second Dialog**:
    - **Source options**: `Ros topics`
    - **Topic**: Select the target camera topic (e.g., `/sensing/camera/camera0/image_raw`).
    - **QoS Reliability**: `BEST_EFFORT`
    - **QoS Durability**: `VOLATILE`
    - Click **Ok**.
    ![Second Dialog](images/2nd_diag.png)

### Step 2: Main Window Configuration
In the Main Window, adjust the following settings:

1.  **Visualization options**:
    - Check **Draw training occupancy**.
    - Set **Drawings alpha** to `0.3`.
2.  **Calibration parameters**:
    Set the coefficients based on the camera's Field of View (FoV).

| Camera Type | Radial Distortion | Rational Distortion |
| :--- | :--- | :--- |
| **30° (C2-30)** | `2` | `0` |
| **120° (C2-120)** | `3` | `3` |

![Calibration Parameters](images/image-20241007-023441.png)

### Step 3: Data Collection & Calculation
1.  **Move the Chessboard**: Move the board slowly to cover the entire field of view. Aim to turn all occupancy cells red.
2.  **Calibrate**: Click **Calibration control** > **Calibrate**.
3.  **Save Results**: Once status is "idle", click **Save**.
    - Select a temporary folder (e.g., `/tmp/calib/camera<N>`).
    ![Save Results](images/image-20241007-042940.png)

---

## 3. Evaluation & Refinement

### Step 1: Confirm the Result (Evaluation Mode)
1. Execute the tool again but select **Image files** in the first dialog.
2. Click **Load Intrinsics** and select the saved `*_info.yaml`.
3. Select the images in the `evaluation_images/` folder.
4. Set **Image view type** to `Source rectified`.
5. **Success Criteria**: The rectified image should look roughly symmetric. If it looks distorted (e.g., asymmetric X-shape), you need to refine the data.

### Step 2: Refinement
If the results are poor:
1.  Inspect the `training_images` folder.
2.  **Remove Bad Samples**: Delete images with motion blur or where the board is not clearly detected.
3.  **Recalibrate**: Run the tool using **Image files** as the source, selecting the cleaned `training_images` folder, and repeat the calibration.

---

## 4. Applying the Results

### Step 1: Edit the YAML File
Open the saved `<camera_name>_info.yaml` and apply these manual changes:

```yaml
# 1. Set the camera name
camera_name: "camera<N>"  # e.g., camera0

# 2. For 120° cameras only:
distortion_model: "rational_polynomial"
# Ensure the D matrix has 8 columns (add zeros if necessary)
```

### Step 2: Deploy to ECUs
Copy the finalized `camera_info.yaml` to the appropriate location on the ECUs.

| Component | Destination Path |
| :--- | :--- |
| **Path** | `data_recording_system/src/individual_params/config/default/camera<N>/camera_info.yaml` |

**ECU Mapping:**
- **ECU0**: `camera0`, `camera1`, `camera2`, `camera3`
- **ECU1**: `camera4`, `camera5`, `camera6`, `camera7`

---

**Next Step**: [Camera-LiDAR extrinsic calibration](extrinsic_calibration.md)
