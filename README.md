# unknown-pick

**End-to-end Solution for Robotic Manipulation of Unknown Objects**

Miaoding Dai
> Final Project in MSR, Northwestern University (Fall 2018); advised by [Argallab](http://www.argallab.northwestern.edu/).

Demo video can be found [here](https://youtu.be/AsTXIjYesws). Refer to my [project presentation](https://drive.google.com/file/d/1nkwlLNjLoMK-qR8m-gktYasgBhdgwJU_/view?usp=sharing) and [portfolio](https://mdai17.github.io/unknown_pick.html) for more details.

## Pipeline Flow

![pipeline](./doc/image/workflow.png)

#### Overview

All parts within blue rectangles are highly independent and can be replaced by alternatives, as long as the inputs/outputs are the same or similar.

Currently, [Mask R-CNN](https://github.com/matterport/Mask_RCNN) is integrated for the ***Perception*** part; two implementations ([naive pose detection](https://github.com/zjudmd1015/unknown-pick/blob/master/src/gen_naive_pose.cpp) or [GPD](https://github.com/atenpas/gpd)) are integrated for ***Grasp Detection*** part; MoveIt! is used for robot arm/gripper path ***plan and execution***.

## Dependencies

- ROS Kinetic (Ubuntu16.04)
- MoveIt!
- PCL1.7 (apt-get)
- OpenCV (2.4.9)
- mico_base (private repo in [Argallab](argallab.northwestern.edu))
- [iai_kinect2](https://github.com/code-iai/iai_kinect2): Kinect V2 driver ROS wrapper
- [Mask R-CNN](https://github.com/matterport/Mask_RCNN): for **Perception** part
- [vision_opencv](https://github.com/ros-perception/vision_opencv): provides **cv_bridge** for Python3
- ros-kinetic-aruco (apt-get): for **Calibration**
- [GPD](https://github.com/atenpas/gpd): for **Grasp Detection** part (optional)

## Usage

#### Preparation
1. ROS network
    1. If you are using it only on lab computer, no need to set up ROS network;
    2. Otherwise, set it properly;
2. Move Mico to a proper position, open Mico;
3. Connect Kinect V2 to computer.

#### Calibration
As RGB-D sensor is mounted on the side of Mico (different from that being mounted on hand), which is called 'eye-on-base', we need to calibrate rigid transfromation from ‘camera_frame’ to ‘mico_base’.

We use [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) package for calibration, which is super fast and convenient, and has already been integrated in **mico_base** package. All setup for this pipeline has been wrapped in the [mico_kinect_calibration.launch](https://github.com/zjudmd1015/unknown-pick/blob/master/launch/mico_kinect_calibration.launch) launch file. If you are using Kinect V2, you can just directly use it.

`$ roslaunch unknown_pick mico_kinect_calibration.launch`

To calibrate, run the command above and follow GUI to finish calibration. Stop calibration if *compute* button gives almost the same result. Usually it takes 60 pictures or so to give a confident calibration result. **Put the calibration result in **demo.launch** file **tf_cam** argument. **

#### Run pipeline
Here, I will talk about how to run code through ROS network, where hardware related code is running in lab computer while other code is running in connected laptop. Running code on only one computer is just almost the same.

In lab computer (master),

1. `$ roscore`
2. `$ rosrun mico_interaction mico_hardware.launch`
3. `$ rosservice call /mico_interaction/switch_controllers 'trajectory'`

In laptop (slave),

1. First, open mask_generator service in python virtualenv 'mask_rcnn' (which provides with python environment for Mask R-CNN). This need to be launched separately since ROS are running in different version of Python (2 vs 3).
`$ roslaunch unknown_pick mask_generator.launch`

2. Then, set arguments in the **most important** launch file **demo.launch**, choose the configuration you want the pipeline to run with, then just run it.
`$ roslaunch unknown_pick demo.launch`