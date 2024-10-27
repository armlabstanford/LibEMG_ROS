#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String
from libemg.streamers import oymotion_streamer
from libemg.data_handler import OnlineDataHandler
from libemg.data_handler import OfflineDataHandler
from libemg.utils import make_regex
from libemg.feature_extractor import FeatureExtractor
from libemg.emg_classifier import OnlineEMGClassifier, EMGClassifier
import signal
import rospkg
import os
import sys
import datetime

class Classifier:
    def __init__(self, filename):
        oymotion_streamer()
        self.odh = OnlineDataHandler(file=True, file_path=filename)
        self.odh.start_listening()

        # Socket for reading EMG
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        self.sock.bind(('127.0.0.1', 12346))

        WINDOW_SIZE = 100 
        WINDOW_INCREMENT = 50

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('libemg_ros')

        # Step 1: Parse offline training data
        dataset_folder = os.path.join(package_path, 'data/')
        classes_values = ["0","1","2","3","4"]
        classes_regex = make_regex(left_bound = "_C_", right_bound=".csv", values = classes_values)
        reps_values = ["0", "1", "2"]
        reps_regex = make_regex(left_bound = "R_", right_bound="_C_", values = reps_values)
        dic = {
            "reps": reps_values,
            "reps_regex": reps_regex,
            "classes": classes_values,
            "classes_regex": classes_regex
        }

        offline_dh = OfflineDataHandler()
        offline_dh.get_data(folder_location=dataset_folder, filename_dic=dic, delimiter=",")
        train_windows, train_metadata = offline_dh.parse_windows(WINDOW_SIZE, WINDOW_INCREMENT)

        # Step 2: Extract features from offline data
        fe = FeatureExtractor()
        feature_list = fe.get_feature_groups()['HTD']
        training_features = fe.extract_features(feature_list, train_windows)

        # Step 3: Dataset creation
        data_set = {}
        data_set['training_features'] = training_features
        data_set['training_labels'] = train_metadata['classes']

        # Step 4: Create the EMG Classifier
        o_classifier = EMGClassifier()
        o_classifier.fit(model="LDA", feature_dictionary=data_set)

        # Step 5: Create online EMG classifier and start classifying.
        self.classifier = OnlineEMGClassifier(o_classifier, WINDOW_SIZE, WINDOW_INCREMENT, self.odh, feature_list)
        self.classifier.run(block=False) # block set to false so it will run in a seperate process.

        self.running = True
        rospy.init_node('emg_publisher', anonymous=True)
        self.publisher = rospy.Publisher('emg', String, queue_size=10)
        self.window_size = 3
        self.cache = []
        self.input_class = 2

    def handle_emg(self):
        data, _ = self.sock.recvfrom(1024)
        data = str(data.decode("utf-8"))
        if data:
            input_class = float(data.split(' ')[0])
            self.cache.append(input_class)
            # pop old ones
            if len(self.cache) > self.window_size:
                self.cache.pop(0)
            ### check for majority class
            ## input_class = max(set(self.cache), key = self.cache.count)
            # check if all entries in cache are the same
            # update the message only if more than 3 times same prediction
            if len(set(self.cache)) == 1:
                self.input_class = self.cache[0]
            else:
                pass # keep the previous input_class   
            if self.input_class == 0:
                self.publisher.publish("hand_close")
            elif self.input_class == 1:
                self.publisher.publish("hand_open")
            elif self.input_class == 2:
                self.publisher.publish("no_motion")
            elif self.input_class == 3:
                self.publisher.publish("wrist_extension")
            elif self.input_class == 4:
                self.publisher.publish("wrist_flexion")
            else:
                return
            
    def stop_running_game(self, filename):
        self.running = False
        self.odh.stop_listening()
        rospy.signal_shutdown("Game Ended")

    def run_game(self):
        signal.signal(signal.SIGINT, self.stop_running_game)
        while self.running: 
            self.handle_emg()

if __name__ == "__main__":
    print(sys.argv)
    # if no arguments are given, use default values
    if len(sys.argv) < 4:
        participant = "0"
        mode = "0"
        trial = "0"
    else:
        participant = sys.argv[1]
        mode = sys.argv[2]
        trial = sys.argv[3]
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    filename = "../data/participant" + participant + "-" + "mode" + mode + "-" + "trial" + trial + "_" + timestamp + "_EMG.csv"
    classifier = Classifier(filename)
    classifier.run_game()