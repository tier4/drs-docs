# Sensor operation check

Before starting the calibration process, verified that all sensors are functioning correctly and that their data is accessible from the calibration PC.

---

## 1. Camera Check

| Camera ID | Position |
| :--- | :--- |
| **camera0** | Front Narrow |
| **camera1** | Front Wide |
| **camera2** | Right Front |
| **camera3** | Right Rear |
| **camera4** | Rear Narrow |
| **camera5** | Rear Wide |
| **camera6** | Left Rear |
| **camera7** | Left Front |

### Step 1: Visualize Images (PC Side)

On the calibration PC, use `rqt_image_view` to verify the streams.

> [!TIP]
> If you are using Docker, remember to run this command **inside the calibration container**.

> [!NOTE]
> Since you are viewing uncompressed image data (`image_raw`), the display frame rate may be lower than the actual capture frame rate due to network bandwidth and processing overhead.

```bash
ros2 run rqt_image_view rqt_image_view
```

**Checklist:**
- [ ] **Image Stream**: An image is displayed correctly for each camera.
- [ ] **Topic Name**: Choose `/sensing/camera/camera<N>/image_raw` (not the compressed one).
- [ ] **Mapping**: The camera ID in the topic name corresponds to the physical mounting position on the vehicle.

![Camera Check](images/image-2025-09-01-17-15-08.png)

---

## 2. LiDAR Check

Verify that all LiDARs are publishing point cloud data and their coordinate frames (TF) are correctly aligned. It is assumed that the DRS sensor services are already running on the ECUs.

### Step 1: Launch Decoder (PC Side)

On the calibration PC, launch the point cloud decoder. If using Docker, run this in the **runtime container**.

```bash
# Enable TF publishing for visualization
ros2 launch drs_launch drs_offline.launch.xml publish_tf:=true
```

> [!NOTE]
> If you are using the Seyond LiDAR driver, add the argument `lidar_driver_type:=seyond` to the launch command.

### Step 2: Visualize in RViz2 (PC Side)

Launch RViz2 and configure the displays.

> [!TIP]
> If you are using Docker, remember to run this command **inside the calibration container**.

```bash
rviz2
```

**Required RViz2 Configuration:**

> [!NOTE]
> Depending on the driver used, the topic name may be `seyond_points` instead of `nebula_points`.

1.  **Global Options**: Set `Fixed Frame` to `base_link`.
2.  **PointCloud2 Display**: Add a display for front/right/rear/left LiDAR topics (e.g., `/sensing/lidar/front/nebula_points`).
    -   **Reliability Policy**: `Best Effort`
    -   **Color Transformer**: `FlatColor` (assign a different color to each LiDAR for easy identification).

**Checklist:**
- [ ] **Point Cloud Display**: Data is visible for each LiDAR.
- [ ] **Physical Matching**: The position of the point cloud in 3D space matches the physical mounting position.

![LiDAR Check](images/image-2025-09-01-17-37-09.png)

---

**Next Step**: [Camera intrinsic calibration](intrinsic_calibration.md)
