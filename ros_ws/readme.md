# Pose Estimation (Jetson Nano)

```bash
$ cd ~/Project/NCS-con/ros_ws
$ catkin_make 
```

## Usage

### Calibration
The calibration file should be arranged as follows
```
calibration_files
├── 1
│   └── cali_640x480.yaml
├── 2
│   └── cali_640x480.yaml
└── 3
    └── cali_640x480.yaml
```

### Launching Basic Services
1. Go to the root folder of ros project
    ```bash
    $ cd ~/Project/NCS-con/ros_ws
    $ roscore
    ```
2. Open ssh to raspberry pis and in a tmux session
3. Launch the marker publisher
    ```bash
    $ rosrun ncs_con ncs_con_node 
    ```
4. Launch the image_extraction services. Go to the sessions opened in step 2 and execute
    ```bash
    $ python3 image_extraction.py
    ```

### Initialization 
Once the positions or orientations of are changed, the re-initialization is required.

Make sure the image extraction services running and then.
1. Launching the recorder
    ```bash
    $ rosrun ncs_con ncs_con_record <path_to_calibrations>
    ```
    where `<path_to_calibrations>` indicates the folder path in Calibration step. 
    After this step, a new file `aruco.detection` should be generated in `<path_to_calibrations>`.
2. Running the initializer. This step is highly suggested not to be conducted on the Jetson Nano, otherwise it might take several hours to finish. (You can copy the entire `<path_to_calibrations>` to another computer)
    ```bash
    $ rosrun ncs_con ncs_con_initialize <path_to_calibrations> <size_of_markers>
    ```
    where `<size_of_markers>` indicates the width of the ArUco markers unit in meters.
    After the initialization is finished, several files will be generated in `<path_to_calibrations>`, where the `final.solution` will be used in tracking step.

### Tracking
Make sure the image extraction services running and then run
```bash
$ rosrun ncs_con ncs_con_track <path_to_calibrations>
```