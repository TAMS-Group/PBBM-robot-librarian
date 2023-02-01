# The polygon of the detected spine
int32[] polygon

# The estimated size of the spine in metres
float32[] spine_size

# The source image of the detected spine
sensor_msgs/Image spine

# The recognized text fragments
std_msgs/String[] text

# The confidence of the recognized text fragments
float32[] confidences

# The source image for all text detections
sensor_msgs/Image[] sources

# The pose of the detected book (center of spine)
geometry_msgs/PoseStamped pose

# The offset of the detected spine towards the shelf its standing on
float32 offset

# the histogram of the hue channel from the hsv converted spine
float32[] hue_histogram