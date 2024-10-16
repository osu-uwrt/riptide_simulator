# Camera Faker

## Overview
The `camera_faker` package is part of `riptide_simulator` and contains a node to output realistic underwater color and depth camera data to allow for full system integration testing in simulation. The package uses OpenGL to render the competition pool environment and task objects in real time. The node interfaces with the physics engine to update the robot's cameras position, which views the scene.

## Common Configuration
The common configuration for this package is specified in `include/settings.h` and includes parameters used by the node and graphics settings.

### Configuration Parameters
| Parameter Name    | Type                | Description                                             |
| ----------------- | ------------------- | ------------------------------------------------------- |
| `yolo_model_path` | `string`            | Path to the YOLO model file.                            |
| `class_id_map`    | `dict<int, string>` | Mapping of class IDs to their respective names.         |
| `threshold`       | `float`             | Confidence threshold for object detection.              |
| `iou`             | `float`             | Intersection over Union threshold for object detection. |

## Usage
#### Launching
The node in this package is launched individually using
```bash
ros2 launch camera_faker zedfaker.launch.py
```
*Note: The physics engine must be running before it can start.*

There are several other launch options that can do the entire robot bring up with the physics engine in one command. Idk what they are though :P
#### Modifying Object's Position, Size, or Image
Go in the `objectSetup()` within `zed_faker.cpp` and edit the inputs to the `Object` constructor parameters or create an additional `Object` and push it to the back of the `objects` vector to include it. Remember to save and build to see changes.
```bash
colcon build --packages-select camera_faker
```
#### April Tag
To toggle a display for the April Tag, press `T` in the OpenGL window.

## Diagnostics and Monitoring

- **View Published Output**: Monitor and visualize the fake camera output using the ROS tool  `rqt`. To run the program, enter the command
	```bash
	rqt
	```
	To view the messages, go to Plugins ➡ Visualization ➡ Image View. With the `camera_faker` running, refresh the topics and click the drop down to select `/talos/zed/zed_node/depth/depth_registered` to view the depth images or `/talos/zed/zed_node/left/image_rect_color` to view the color images. `rqt` can also be used to see the vision detections on top of the fake images.
- **View Scene and Robot**: Within the OpenGL window, press `Tab` or `F5` to switch to a user controlled fly around observation camera. The robot's perspective will continue to be published regardless of what mode is being display in the OpenGL window. To control the camera, look by moving the mouse in the window,  and use `WASD` and `Shift/Space` for movement. 
- **View Debug Info**: While in  the fly around camera mode, press `F3`to see a debug screen contain frame rate, camera positions, and ROS time.

## Troubleshooting
### Problem: Stuck on UWRT Splash Screen
- **Solution**: Ensure the physics engine is running, it is likely waiting from TF frames. Otherwise, check the terminal output to see if there was any shader errors or any issues opening assets.

### Problem: Poor Frame Rate
- **Solution**: Ensure it is being run on a Native Linux install and not a Virtual Machine or WSL. Additionally, avoid sending the published raw image data across devices on the ROS network, that eats up bandwidth. 

### Problem: Black OpenGL Window
- **Solution**: Something was modified in the code that made OpenGL crash. Go back to a previous working revision and add back pieces one at a time to find the culprit. OpenGL is notoriously bad at being unforgiving for debugging issues. One reason this could happen is calling OpenGL functions from a constructor before OpenGL was initialized.

### Problem: Vision Detection Giving Incorrect Positions
- **Solution**: Check to make sure the camera's intrinsic parameters are correct in the settings. These parameters are used by the in `riptide_perception` to calculate the position from the image. The parameters would need to be changed if the `IMG_WIDTH` or `IMG_HEIGHT` settings are modified.

## Node

### Zed Faker Node

#### Overview
The `zed_faker` node utilizes an OpenGL graphics system to render real-time depth and color camera data. 

#### ROS2 Interfaces

**Parameters**
| Parameter Name   | Type     | Default | Description                                             |
| ---------------- | -------- | ------- | ------------------------------------------------------- |
| `robot`          | `string` | `talos` | Name of the AUV being simulated.                        |
| `shader_folder`  | `string` | N/A     | Directory containing all vertex and fragment shaders.   |
| `texture_folder` | `string` | N/A     | Directory containing all texture assets.                |
| `model_folder`   | `string` | N/A     | Directory containing all 3D models.                     |
| `font_folder`    | `string` | N/A     | Directory containing all font files for text rendering. |

**Subscriptions**
None.

**Publishers**
| Publisher Topic                              | Message Type             | Description                            |
| -------------------------------------------- | ------------------------ | -------------------------------------- |
| `/talos/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image`      | Publishes raw depth image.             |
| `/talos/zed/zed_node/left/image_rect_color`  | `sensor_msgs/Image`      | Publishes raw color images.            |
| `/talos/zed/zed_node/left/camera_info`       | `sensor_msgs/CameraInfo` | Publishes intrinsic camera parameters. |
| `/talos/zed/zed_node/depth/camera_info`      | `sensor_msgs/CameraInfo` | Publishes depth camera info.           |

#### Functional Description

  1. **Frame Buffer Object (FBO)**:
	   - A buffer that can hold images which can be rendered to
	   - The default FBO displays to images tp the screen
3. **Vertex Shaders**:
   - Contains information belonging to a vertex (like position, color, texture coordinates)
   - Sends information to `Fragment Shaders`.
3. **Fragment Shaders**:
   - Interpolates vertex data
   - For each pixel, the GPU computes in parallel what the color of each pixel should be based on vertex data and other variables called `uniforms`
   
  
  

#### Key Functions and Classes
| Function/Class                                                      | Description                                                  |
| ------------------------------------------------------------------- | ------------------------------------------------------------ |
| `initialize_yolo(yolo_model_path)`                                  | Initializes the YOLO model.                                  |
| `camera_info_callback(msg: CameraInfo)`                             | Processes camera info messages.                              |
| `depth_info_callback(msg: CameraInfo)`                              | Processes depth camera info messages.                        |
| `depth_callback(msg: Image)`                                        | Processes depth images.                                      |
| `image_callback(msg: Image)`                                        | Processes RGB images, runs detection, and publishes results. |
| `create_detection3d_message(header, box, cv_image, conf)`           | Creates detection messages.                                  |
| `publish_marker(quat, centroid, class_id, bbox_width, bbox_height)` | Publishes visualization markers.                             |
| `publish_accumulated_point_cloud()`                                 | Publishes accumulated point clouds.                          |
