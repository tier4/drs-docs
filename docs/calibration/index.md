# DRS Calibration

This document provides a guide for calibrating the sensors used in the Data Recording System (DRS).

## Calibration Workflow

Please follow the steps below in order:

1.  **[Setup](setup.md)**
    Prepare the environment using Docker or by building from source.
2.  **[Sensor operation check](sensor_check.md)**
    Verify that the camera and LiDAR are functioning correctly.
3.  **[Camera intrinsic calibration](intrinsic_calibration.md)**
    Calibrate the internal parameters for each camera.
4.  **[Camera-LiDAR extrinsic calibration](extrinsic_calibration.md)**
    Calibrate the relative pose between the camera and LiDAR.
5.  **[LiDAR-LiDAR calibration](lidar_calibration.md)**
    Calibrate the relative pose between multiple LiDARs.
6.  **[Result integration and Application](result_integration.md)**
    Generate the final TF configuration and apply it to the ECUs.
