# Exercise 4: Apriltag Detection & Safety on Robots

This folder contains the implementation for Exercise 4

---

## File Structure

```
.
├── packages
│   ├── camera_package
│   │   └── apriltag_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting apriltag]
│   │   └── apriltag_driving_node
│   │       └── [DESCRIPTION: Node responsible for lane following with stop and set LED based on apriltag]
│   │   └── camera_distortion_node
│   │       └── [DESCRIPTION: Node responsible for recovering the orignal image without distortion]
│   │   └── color_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting red line on lane]
│   │   └── lane_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting yellow and white lines]
│   │   └── lane_driving_node
│   │       └── [DESCRIPTION: Node responsible for lane following (as a template)]
│   ├── crosswalk_package
│   │   └── crosswalk_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting duckies, blue line and updating the states according to the detected objects]
│   │   └── crosswalk_driving_node
│   │       └── [DESCRIPTION: Node responsible for lane following with stop depending on the states]
│   │   └── lane_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting yellow and white lines for PID error]
│   │   └── camera_distortion_node
│   │       └── [DESCRIPTION: Node responsible for recovering the orignal image without distortion]
│   ├── navigation_package
│   │   └── lane_detection_node
│   │       └── [DESCRIPTION: Node responsible for detecting yellow and white lines for PID error]
│   │   └── camera_distortion_node
│   │       └── [DESCRIPTION: Node responsible for recovering the orignal image without distortion]
│   │   └── safe_navigation_node
│   │       └── [DESCRIPTION: Node responsible for detecting duckiebots and updating the stages for manuvering]
│   │   └── navigation_driving_node
│   │       └── [DESCRIPTION: Node responsible for motion behaviour such as lane following, turning and stopping]
│   └── template_package
│       └── template_node
│           └── [DESCRIPTION: Node responsible for ...]
└── launchers
    ├── apriltag_detection.sh
    │   └── [DESCRIPTION: Launcher script for apriltag detection (debug)]
    ├── apriltag_driving.sh
    │   └── [DESCRIPTION: Launcher script for part 1 with lane following]
    ├── camera_distortion.sh
    │   └── [DESCRIPTION: Launcher script for correcting image (debug)]
    ├── color_detection.sh
    │   └── [DESCRIPTION: Launcher script for color lines detection (debug)]
    ├── lane_detection.sh
    │   └── [DESCRIPTION: Launcher script for yellow and white line detection (debug)]
    ├── lane_driving.sh
    │   └── [DESCRIPTION: Launcher script for lane following (debug)]
    ├── apriltag_detection.sh
    │   └── [DESCRIPTION: Launcher script for apriltag detection (debug)]
    ├── crosswalk_detection.sh
    │   └── [DESCRIPTION: Launcher script for part 2 with lane following]
    └── safe_navigation.sh
        └── [DESCRIPTION: Launcher script for part 3 with lane following]
```
