from tkinter import *
from classify import HandGame
from libemg.screen_guided_training import ScreenGuidedTraining
from libemg.data_handler import OnlineDataHandler, OfflineDataHandler
from libemg.streamers import oymotion_streamer
from libemg.utils import make_regex
from libemg.feature_extractor import FeatureExtractor
from libemg.emg_classifier import OnlineEMGClassifier, EMGClassifier

class Menu:
    def __init__(self):
        # Myo Streamer - start streaming the myo data 
        oymotion_streamer(platform='Linux')

        # Create online data handler to listen for the data
        self.odh = OnlineDataHandler()
        self.odh.start_listening()

        self.classifier = None

        # UI related initialization
        self.window = None
        self.initialize_ui()
        self.window.mainloop()

    def initialize_ui(self):
        # Create the simple menu UI:
        self.window = Tk()
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.title("Game Menu")
        self.window.geometry("500x400")

        # Label 
        Label(self.window, font=("Arial bold", 20), text = 'LibEMG - Hand Demo').pack(pady=(10,20))
        # Train Model Button
        Button(self.window, font=("Arial", 18), text = 'Train Model', command=self.launch_training).pack(pady=(0,20))
        # Classify Button
        Button(self.window, font=("Arial", 18), text = 'Classify', command=self.play_game).pack()

    def play_game(self):
        self.window.destroy()
        self.set_up_classifier()
        HandGame().run_game()
        # Its important to stop the classifier after the game has ended
        # Otherwise it will continuously run in a seperate process
        self.classifier.stop_running()
        self.initialize_ui()

    def launch_training(self):
        self.window.destroy()
        # Launch training ui
        training_ui = ScreenGuidedTraining()
        training_ui.download_gestures([1,2,3,4,5], "images/")
        training_ui.launch_training(self.odh, 2, 3, "images/", "data/", 1)
        self.initialize_ui()

    def set_up_classifier(self):
        WINDOW_SIZE = 100 
        WINDOW_INCREMENT = 50

        # Step 1: Parse offline training data
        dataset_folder = 'data/'
        classes_values = ["0","1","2","3","4"]
        classes_regex = make_regex(left_bound = "_C_", right_bound="_EMG.csv", values = classes_values)
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

    def on_closing(self):
        # Clean up all the processes that have been started
        self.odh.stop_listening()
        self.window.destroy()

if __name__ == "__main__":
    menu = Menu()
