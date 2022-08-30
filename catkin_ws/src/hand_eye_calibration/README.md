# Hand Eye Calibration

## Step

**Make sure let apriltag can see in the D435.**

1. Run connect vx300s and control.
    ```
        roslaunch vx300s_bringup vx300s_control.launch
    ```
2. Open realsense camera D435.
    ```
        roslaunch realsense2_camera rs_rgbd.launch
    ```
3. Execute apriltag ros to detect.
    ```
        roslaunch hand_eye_calibration cali_cam.launch
    ```
4. Stick fake apriltag to arm.
    ```
        roslaunch hand_eye_calibration fake_tcp_publisher.launch
    ```
5. Auto move arm to collect point.
    ```
        rosrun hand_eye_calibration static_hand_eye_calibration _file_name:=[filename]
    ```
6. Get the transform matrix.
    ```
        rosrun hand_eye_calibration get_transform src/hand_eye_calibration/data/[filename].txt
    ```
7. Then add static_transform_publisher to your launch file.
8. Finally, open Rviz to see calibration result.