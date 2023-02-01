# Overview

This repository contains the code for the vision pipeline of the librarian masters project. 

There are two packages:
|Package|Description|
|-|-|
| `librarian_vision_node` | Contains all relevant nodes to run the vision |
| `librarian_panel` | Features an rviz panel to monitor the published environment message |

Both packages depend on `librarian_resources` make sure to have it in the workspace as well.

## Librarian Vision Node

The `librarian_vision_node` package makes up almost the entire vision pipeline used to recognize books in the shelf in order to manipulate them. Since the books have no special markings (e.g. april tags) the perceptions is rather difficult.

In total, there are 7 nodes in this package:
1. `librarian_image_preproc.py`
2. `detection_node.py`
3. `recognize_node.py`
4. `librarian_book_extract.py`
5. `librarian_pc_process_node`
6. `librarian_environment.py`
7. `book_cover_inspection_node.py`

### 1. librarian_image_preproc.py
This node runs the image preprocessor by applying a perspective transform twice on the camera image. The result are two images for each shelf level with corrected perspective. The idea is two have the books sides being aligned with the image axis. This makes the spine detection more precide as the bounding boxes returned by YOLO are parallel to the image axes as well. Another advantage of the transformation is that unnecessary parts of the image (e.g. the top of the shelf) are cropped out.

### 2. detection_node.py
This node uses the two preprocessed shelf images to detect book spines and text. The book spines are detected with a fine-tuned version of [YOLOv5](https://github.com/ultralytics/yolov5). The text is detexted using the [CRAFT](https://github.com/clovaai/CRAFT-pytorch) network. Because the detections are made using the corrected images, all pixel coordinates of bounding boxes / polygons are then projected back onto the original image. The nodes output are polygons of detected book spines and text.

### 3. recognize_node.py
The recognize node aims to _read_ the detected text coming from the detection node. It uses either the [DTRB](https://github.com/clovaai/deep-text-recognition-benchmark) model or the [google cloud vision api](https://cloud.google.com/vision/docs/ocr) for text recognition. The text fragments are passed along with the detected spines.

### 4. librarian_book_extract.py
This node projects the book spine detections into 3D space by determining the pose and dimensions in metres. To do so, it uses the `librarian_pc_process_node` to access the point cloud. Furthermore, it assigns each detection a book id by matching them against the `books.csv` database.

### 5. librarian_pc_process_node
This offsers a service to get the average position of given points defined by screen coordinates. This allows to compute a book spines position given all pixels that are inside the polygon (which is in screen coordinates).

### 6. librarian_environment.py
This node continuously receives observations made by all previous steps and accumulates them. The temporal sequence of book sets is then reduced to one belief about the environment. The book positions are clustered with k-Means and the ids are greedily assigned. Pose and dimensions are averaged over each book from the cluster. The constructed environment is finally handed to the `interaction_node.py` residing in the `librarian_resources` package.

### 7. book_cover_inspection_node.py (unused)
This node offsers a service to inspect the closup of the currently grasped book being held in front of the camera. It uses the vanilla version of [YOLOv5](https://github.com/ultralytics/yolov5) to find the book and crop out any other contents from the camera image. The covers text is then read using CRAFT and DTRB again. The text fragments are used to determine the most like candidate from the database. This candidate is then returned along with the read text, the title from the candidate, and a confidence. So far, it is not used in the final implementation in the project.   

## Librarian Panel
This rviz plugin shows the latest emitted environment in the shape of a table. It is soley used for monitoring.

![Screenshot from the librarian rviz panel](images/rviz_panel.png)

# Usage Requirements
The project was build with ROS noetic.

After cloning this repository into your catkin workspace make sure that your system meets the following requirements:

1. librarian_common
2. apriltag + apriltag_ros
3. moveit planning scene
4. CUDA compatible GPU + CUDA drivers
5. network weights of finetuned Yolov5, CRAFT, and DTRB
6. The [point cloud library](https://pointclouds.org/)
7. additional python packages

## 1. librarian_common
The librarian_common package contains message/service definitions, the `librarian_shelf_from_image.py` node, images from book spines, and the book database. Even when running _just_ the vision, this package needs to be build and available for the vision.

## 2. Apriltag + apriltag_ros
The apriltag detection packages can be found under:
[https://github.com/AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag_ros), and
[https://github.com/AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)

## 3. Moveit planning scene
The vision does not depend on a certain robot but it needs to access the moveit planning scene interface. To configure the vision pipeline look at the section _Launch files_.

## 4. CUDA compatible GPU + CUDA drivers
The vision pipeline makes use of deep neural networks implemented in pytorch. It was **not** tested without GPU acceleration and may be impossible to run.

## 5. Network weights of finetuned Yolov5, CRAFT, and DTRB 

All used weights can be downloaded here: [https://cloud.mafiasi.de/s/y5ewsTng6nQ7jY7](https://cloud.mafiasi.de/s/y5ewsTng6nQ7jY7).

The CRAFT weights were be obtained here: https://drive.google.com/file/d/1Jk4eGD7crsqCCg9C9VjCLkMN3ze8kutZ/view

The DTRB weights were obtained here: https://www.dropbox.com/sh/j3xmli4di1zuv3s/AAArdcPgz7UFxIHUuKNOeKv_a?dl=0

The weights for vanilla YOLOv5 are downloaded automatically.

The weights for the finetuned YOLOv5 are inside the zip file.

### Weight locations
Each weight file has its own place:
| File | Required location |
|-|-|
| `book_spine_model_v1.pt` | librarian_vision/librarian_vison_node/src/librarian_segmentation/ |
| `craft_mlt_25k.pth` | librarian_vision/librarian_vison_node/src/librarian_craft/ |
| `TPS-ResNet-BiLSTM-Attn.pth` | librarian_vision/librarian_vison_node/src/librarian_dtrb/ |

## 6. The point cloud library
Information to install this library can be found here: [https://pointclouds.org/](https://pointclouds.org/)

## 7. Additional python packages
The following python packages are required to run the vision pipeline:

`numpy`, `pandas`, `torch`, `fuzzywuzzy`, `Levenshtein`, `matplotlib`, `pillow`, `pyclustering`, `opencv-python`, `PyKDL`

Optionally, if you attempt to use the google cloud vision api, the python package `google-cloud-vision` needs to be installed and credentials provided.

Please check [https://cloud.google.com/vision/](https://cloud.google.com/vision/) for more information.

### Google cloud vision api setup experience ([quick guide](https://cloud.google.com/vision/docs/detect-labels-image-client-libraries))
* Create Project
* Under the project click Vision API
* Enable API
* Create a service account key
* **Set the environment variable GOOGLE_APPLICATION_CREDENTIALS to the path of the JSON file that contains your service account key. **

# Launch files

There are 4 different launch files in the `librarian_vision_node` package:

| Launch file | Description |
|-|-|
| `librarian_continuous_detection.launch` | Launches the april tag detection with the correct topics |
| `librarian_vision.launch` | Base launch file for the vision pipeline |
| `librarian_vision_pr2_azure.launch` | Launches the vision on the pr2 configured with an azure kinect camera |
| `librarian_vision_pr2_azure_sim.launch` | Same as `librarian_vision_pr2_azure.launch` but sets `/use_sim_time` to `true` to work with ros bags |

To start the vision pipeline on the pr2 launch `librarian_vision_pr2_azure.launch`. This file includes the base launch file `librarian_vision.launch` which on the other hand includes `librarian_continuous_detection.launch`. That is why only one launch file should be necessary to run.

## Overview of topics and parameters
To write your own launch file adapting to a different setup, you can use the `librarian_vision_pr2_azure.launch` as a starting point. Nevertheless, here is a list of topics and parameters that are used by the vision pipeline.

### Topics
Use the `<remap>` command to map from your specific topic to the required one. 

| Topic | Description |
|-|-|
|`/librarian/camera_info`| The `CameraInfo` topic from the camera facing to the bookshelf  |
|`/librarian/input_image`| The (at best rectified) RGB image from the camera facing to the bookshelf |
|`/librarian/points2`| The pointcloud that should display the scene from the same standpoint of the camera |

### Parameters
| Parameter | Description |
|-|-|
| `/librarian/apriltag_image_height` | The height of the scaled camera image being used to detect the april tag |
| `camera_frame` | The source frame used by the april tag detection |

# Build
It should be sufficient to build the packages with catkin: `catkin build`.
 
# Topics
Here is a list of topics with a rough description order by _internal_, _monitoring_, and _unused_. The monitoring topics are a good way to ensure everything works properly. They are meant to be visualized in RViz.  

|Topic|Description|Type|
|-|-|-|
|Internal|||
|`/librarian/apriltag_camera/camera_info`   | Manipulated camera info to work with scaled down images. Used by apriltag detection. | `sensor_msgs/CameraInfo` |
|`/librarian/apriltag_camera/image_color`   | Scaled down camera image. Used by april tag detection.  | `sensor_msgs/Image` |
|`/librarian/book/detections`               | Contains polygons about detected text and book spines.  | `librarian_resources/BookDetections` |
|`/librarian/books`                         | One observation of books containing id, confidence, and recognition data. | `librarian_resources/Books` |
|`/librarian/control/vision`                | Turns the vision pipeline on/off.| `librarian_resources/VisionControl` |
|`/librarian/environment`                   | Accumulated observations reducing noise. Otherwise same as `/librarian/books`. | `librarian_resources/Environment` |
|`/librarian/preproc_input/image_color`     | Unchanged camera image republished with slight delay so the tf frames (from april tag detection) are up-to-date. | `sensor_msgs/Image` |
|`/librarian/shelf_images`                  | Images from both shelf levels with the perspective transform applied. Also the inverse transformation matrix is supplied. | `librarian_resources/ShelfImages` |
|`/librarian/text_recognitions`             | Same as `/librarian/book/detections` but the text detections were replaced with recognitions containing the text + confidence. | `librarian_resources/TextRecognitions` |
|Monitoring|||
|`/librarian/detected_book_poses`           | Pose array of all detected spines. | `geometry_msgs/PoseArray` |
|`/librarian/image`                         | Image to monitor the shelf levels with perspective transformation applied. | `sensor_msgs/Image` |
|`/librarian/marker_array`                  | Shows the predicted book ID near each collision object. | `visualization_msgs/MarkerArray` |
|`/librarian/monitor/books/compressed`      | Which spine detection was classified as which book. | `sensor_msgs/CompressedImage` |
|`/librarian/monitor/heatmap`               | Heatmap of detected text. | `sensor_msgs/Image` |
|`/librarian/monitor/matrix/compressed`     | The score matrix showing the predicted similarity for each detection against each book. | `sensor_msgs/CompressedImage` |
|`/librarian/monitor/polys`                 | The detected text / book spines drawn on top of the source image. | `sensor_msgs/Image` |
|Unused|||
|`/librarian/reset_env`                     | Unused | `std_msgs/Empty` |
