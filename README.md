# LibEMG_ROS
This is a ROS package that publishes continuous control inputs from an OyMotion GForce armband to a ROS topic. For publishing OyMotion data directly to ROS, see [here](https://github.com/oymotion/ros_gforce). This package uses the [LibEMG](https://libemg.github.io/libemg/index.html) library to process the data for use in robotics applications. 

The code is only slightly modified from the snake game demo, which uses the Thalmic Labs Myo armband by default. The Myo armband is no longer available, but fortunately, LibEMG supports the OyMotion band as well. Please see [Example 1](https://libemg.github.io/libemg/examples/snake_example/snake_example.html) from the LibEMG documentation for the original game.

To use this package, clone it into a valid catkin workspace and build it. Then
1. Install [LibEMG](https://libemg.github.io/libemg/index.html).
2. The [GForce python interface](https://github.com/oymotion/gForceSDKPython) requires BluePy, so install [BluePy](https://github.com/IanHarvey/bluepy). 
3. To run everything without root privileges, run 
```
  sudo setcap cap_net_raw+e  <PATH>/bluepy-helper
  sudo setcap cap_net_admin+eip  <PATH>/bluepy-helper
```
as described [here](https://github.com/IanHarvey/bluepy/issues/313#issuecomment-437939172).
4. Start `roscore` in a terminal, and navigate to the package and run `train.py` in a separate terminal.
```
python3 train.py
```

This will open a menu that takes training data for hand open/close, wrist flexion/extension, and no motion (click "Train") and then publishes predicted classes to a ROS topic (click "Classify"). To stream the topic to terminal, 
``` 
rostopic echo /emg
```

The topic name can be changed in `classify.py` where the publisher is defined.

Note: The output-dir option in curl might not work with all versions of curl. To make a quick start more likely without debugging version issues, the training images that would have been downloaded automatically on the first execution of the original script are included here.
