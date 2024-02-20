import rospy
import socket
from std_msgs.msg import String
import signal

class HandGame:
    def __init__(self):
        # Socket for reading EMG
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        self.sock.bind(('127.0.0.1', 12346))
        self.running = True
        rospy.init_node('emg_publisher', anonymous=True)
        self.publisher = rospy.Publisher('emg', String, queue_size=10)
        self.window_size = 8
        self.cache = []
    
    def handle_emg(self):
        data, _ = self.sock.recvfrom(1024)
        data = str(data.decode("utf-8"))
        if data:
            input_class = float(data.split(' ')[0])
            self.cache.append(input_class)
            # pop old ones
            if len(self.cache) > self.window_size:
                self.cache.pop(0)
            # check for majority class
            input_class = max(set(self.cache), key = self.cache.count)
            if input_class == 0:
                self.publisher.publish("hand_close")
            elif input_class == 1:
                self.publisher.publish("hand_open")
            elif input_class == 2:
                self.publisher.publish("no_motion")
            elif input_class == 3:
                self.publisher.publish("wrist_extension")
            elif input_class == 4:
                self.publisher.publish("wrist_flexion")
            else:
                return

    def stop_running_game(self):
        self.running = False
        rospy.signal_shutdown("Game Ended")

    def run_game(self):
        signal.signal(signal.SIGINT, self.stop_running_game)
        while self.running: 
            self.handle_emg()