﻿# LibEMG-ROS
This is a ROS package that publishes continuous control inputs from an OyMotion GForce armband to a ROS topic. For publishing OyMotion data directly to ROS, see [here](https://github.com/oymotion/ros_gforce). This package uses the [LibEMG](https://libemg.github.io/libemg/index.html) library to process the data for use in robotics applications. 

It is a modification of the snake game demo, which uses the Thalmic Labs Myo armband by default. The Myo armband is no longer available, but fortunately, LibEMG supports the OyMotion band as well. Please see [Example 1](https://libemg.github.io/libemg/examples/snake_example/snake_example.html) from the LibEMG documentation for the original game.

To use the GForce armband, plug in the bluetooth dongle and switch the armband on while charged.

To use this package, clone it into a valid catkin workspace and build it (not really necessary, but might help keep everything together if there is more development in ROS around it). Then
1. Install [LibEMG](https://libemg.github.io/libemg/index.html) v0.0.1 with `pip3 install libemg==0.0.1`. Version 1.0.2 (latest at the time of this writing) does not support the OyMotion band.
2. The [GForce python interface](https://github.com/oymotion/gForceSDKPython) requires BluePy, so install [BluePy](https://github.com/IanHarvey/bluepy). 
3. To run everything without root privileges, set capabilities as described [here](https://github.com/IanHarvey/bluepy/issues/313#issuecomment-437939172).
```
  sudo setcap cap_net_raw+e  <PATH>/bluepy-helper
  sudo setcap cap_net_admin+eip  <PATH>/bluepy-helper
```
4. Install other dependencies:
```
sudo apt-get install python3-pil python3-pil.imagetk python3-tk
pip3 install pygame


```
5. Start `roscore` in a terminal, and navigate to the package and run `train.py` in a separate terminal.
```
python3 train.py
```

This will open a menu that takes training data for hand open/close, wrist flexion/extension, and no motion (click "Train"), saves it in a /data directory, and then publishes predicted classes to a ROS topic with a GUI we added to check the quality of classification (click "Classify"). The training GUI is the same as in the snake game demo example. A video of training and testing with the GUI is on YouTube [here](https://www.youtube.com/watch?v=Il9N8hQZjtg).

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/videos/emg_train.gif" height="300">    
<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/videos/emg_test.gif" height="300">

To stream the topic to terminal, 
``` 
rostopic echo /emg
```

5. If already trained, the classifier can be run as a rosnode:
```
rosrun libemg_ros classify_rosnode.py
```

The topic name, message contents, and log filename can be changed in `classify_rosnode.py` where the publisher is defined. By default, data will be stored with a timestamp in the filename.

Note: The output-dir option in curl might not work with all versions of curl. To make a quick start more likely without debugging version issues, the training images that would have been downloaded automatically on the first execution of the original script are included here.
