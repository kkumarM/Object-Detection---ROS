# ClearSpot.ai | Fruitpicking: Apple Detection

This ROS Package contains contains Apple Detection and Ripeness Identification Model.

## Installation Instructions:

### Required Packages:
**ROS**: Follow the ROS installation instruction for your system, from the Official Documentation.

This Package has been tested with ROS Noetic, and should be backwards compatible with older distribution like ROS Kinetic, and Melodic. 
#### Python Packages:
```
pip install opencv-python
pip install ultralytics
pip install supervision
```
### Workspace Setup:
NOTE: If you're using an existing Catkin Workspace, replace all mention of ```~/catkin_ws``` with your workspace path.

- Create a new workspace by:
```
mkdir -p ~/catkin_ws/src
```
- Clone/Move the fire_detection ROS Package into the ```src/``` folder:
```
# Cloning into source folder
cd ~/catkin_ws/src/ && git clone <url>

# Or, in the folder containg the package:
mv apple_detection_pkg ~/catkin_ws/src/
```
- Build the workspace, using catkin build:
```
cd ~/catkin_ws/
catkin build
```

## Running Inference

After building the workspace, source it into your ".bashrc" file:
```
echo "~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
In all the cases below, the model publishes an Annotated Image with Predictions and the Prediction Data (bounding boxes, scores etc)
```
Annotated Image: "/detected/annotated_image"
Prediction Data: "/detected/prediction_data"
```
If the Output Topic (by default: "/detected") has been changed in config file, these will change accordingly.

### To run the launch file against a test video
If there's a need to test the launch file against a test video, follow the steps: 
1. Place the Test Video in the ```test/``` folder.
2. Change the ```test_video``` parameter in the ```configs/config.yaml``` file to the video file's name.
    1. (Optional) Other parameters that you can change are the Rate, and Input Topic.
3. Run the ```test_video.launch``` launch file.
```
roslaunch apple_detection_pkg test_video.launch
```
### To run the launch file with a camera feed
1. Edit the ```config.yaml``` file, so that the Input Topic, desired Output Topic & Size are correct.
2. Run the launch file: 
```
roslaunch apple_detection_pkg fire_detection.launch
```

```configs/config.yaml``` has all the parameters for ROS Parameter: Input Topic, Desired Size, Loggin requirements, etc. Change if required.

An Rviz Window will open with two displays, the video feed and the annotated image feed from the model.
