# Herbie the Love Bug

## Team Members

* Cian Woods (cwoods4@jaguarlandrover.com)
* Anirudh Venkata (anirudh.av93@gmail.com)
* M. R. Rahim (dommgr01@gmail.com)
* Glyn Matthews (glyn.matthews@gmail.com)

## Solution

In order to complete the project, we were required to enable the car to travel around the track, stopping when the
traffic lights were red, and continuing again once the traffic lights turned green. The implementation was made on
three components, the traffic light detector (`tl_detector`), the waypoint updater for path planning
(`waypoint_updater`), and drive-by-wire for vehicle control (`twist_controller`).

### Traffic Light Detector
The code for this node can be found in `tl_detector.py` and `tl_classifier.py`.

#### `tl_detector.py`
The traffic light detection node subscribes to four topics: `base_waypoints`, `current_pose`, `image_color` and
`traffic_lights`. `TLDetector` is responsible for finding the nearest traffic light position calculated on
`distanceCalculation()` method and calls `light_classifier.get_classification()` with the current camera image. It
uses the light classifier to get a color prediction. The node then publishes `traffic_waypoints` - the location of any
upcoming red lights for other nodes to control the vehicle.

#### `tl_classifier.py`
For traffic light detection and classification we decided to use an SSD (Single Shot MultiBox Detector) network as the
purpose of an SSD is detect the location and classify the detected object in one pass through the network. This will
improve performance for two reasons:

- Detection and classification are now a single function instead of two operations running on the same image. This
  eliminates and potential duplication of work.
- The network chosen [faster_rcnn_resnet101](http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_lowproposals_coco_2018_01_28.tar.gz)
  is relatively performant running at > 10fps on the students mid tier gpu (Nvidia GTX 1060).

Due to the limited amount of data available to train the network the decision was made to take a pre-trained network
and transfer learn the network on the available simulated and real datasets provided by Udacity. The chosen network was
pre-trained with the COCO dataset.

Transfer learning was achieved using the Object Detection API provided by Tensorflow. For simulated data the network
was trained on the provided data by Udacity, however real data provided by Udacity was supplemented with a dataset of
labelled traffic lights provided by Bosch. This dataset can be found [here](https://hci.iwr.uni-heidelberg.de/node/6132).

### Waypoint Updater

The code for this is entirely in the `waypoint_updater.py` file.

Firstly, there are two callback inputs: `waypoints_cb` sets the base waypoints (these don't change, so they are
only set once) and `traffic_cb` sets the index of the waypoint where the next stop line is. This value is set to the
index of stop line waypoint when the traffic lights immediately ahead are red, and is -1 otherwise.

The main processing loop, in pseudocode, does the following:

```python
for each update:
    index = self.get_closest_waypoint_index()
    lane = self.generate_lane(index)
    self.final_waypoints_pub.publish(lane)
```

For each update, the index of the closest waypoint to the vehicle is computed. From this waypoint, we look 100
waypoints ahead and publish them to the simulator. These waypoints control the speed of the vehicle, and in all normal
circumstances we don't modify the speed (there are no other vehicles and no obstacles in the simulator).

However, we do want to control the speed when the traffic light ahead is red. The the `generate_lane` function updates
the speed value assigned to the waypoints when the vehicle needs to stop at the next stop line. In our implementation,
we simply use the code from the video in the project introduction with only minor modifications.

## Drive-By-Wire

The drive-by-wire node controls the throttle, brake, and steering values of the vehicle. The main updates in the code
are in `dbw_node.py` and `twist_controller.py`.

The inputs to `DBWNode` are the properties of the vehicle, including the mass, fuel capacity and wheel radius. It
continuously publishes the `SteeringCmd`, `BrakeCmd` and `ThrottleCmd`.

The main processing loop does the following:

```python
for each update at 50Hz:
    cte = self.compute_cte()
    throttle, brake, steering = self.controller.control(..., cte)
    self.publish(throttle, brake, steering)
```

The role of the controller is to take the actual vehicle velocity and the expected vehicle velocity, and to compute
the values of the throttle, brake and steering. These values are published to the simulator.

The main processing happens in `twist_controller.py`. Firstly, the actual velocity is filtered using a low pass filter.
The steering value (the yaw) is calculated using the `YawController` provided. A PID controller is also used to
add stability to the steering value. This uses the CTE value compute in the `DBWNode` and passed as an argument to the
`control` function. The throttle is also computed using a PID controller. The brake value is not controlled in
a sophisticated way - a high value (700nm) for the brakes is provided when the vehicle needs to be fixed in place (e.g.
at the stop line). When the vehicle is slowing down, a proportional value of the brakes is computed based on the
total vehicle mass, assuming that the gas tank is full.

## Summary

Using the code in this submission, the vehicle is able to drive in its lane, and to correctly stop at the stop line
when the traffic lights are red. When leaving the simulator run for a long period of time, each of us noted that the
vehicle would eventually leave the track. The precise reason why has proven difficult to pin down, because of system
latency issues. Even when running on the project workspace, it is difficult to find out exactly how to tweak the values
of, e.g., the PID controller co-efficients. Also, the waypoint updater is updated at a rate of only 5Hz. A higher value
is too unstable to test.

By following the walkthroughs in the project introduction, it was possible to build a system that came very close to
"submittable" project. On top of the walkthrough code, we improved the traffic light classification and added some
additional stability to the DBW node.

This project has been very useful to show how to bring all the components we have learnt during the course of a
total self-driving system. While earlier projects were more complex to solve, what we gained from this experience was
how to combine everything together, from perception, to path planning, to vehicle control. The full complexity of
building a self-driving car has become clear. Testing them and tweaking for performance is a huge challenge.

## Installation

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Download trained models from [google drive](https://drive.google.com/open?id=1eEc9RTVZXSub1YJOyDqYlMAPLl32r9UB) and move models into `ros/src/tl_detector/perception/` .
4. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
5. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
