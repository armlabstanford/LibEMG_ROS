import rospy
import socket
from std_msgs.msg import String
import signal
import pygame
from pygame.locals import *

class HandGame:
    def __init__(self):
        # Socket for reading EMG
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        self.sock.bind(('127.0.0.1', 12346))
        self.running = True
        # rospy.init_node('emg_publisher', anonymous=True)
        # self.publisher = rospy.Publisher('emg', String, queue_size=10)
        self.window_size = 8
        self.cache = []

        self.width = 1500
        self.height = 1500

        self.counter = 200

        pygame.init()
        # divide into an upper and lower half, only lower half is updated
        self.window = pygame.display.set_mode([self.width, self.height])
        

        pygame.display.set_caption('Test classifier')
        self.clock = pygame.time.Clock()

        # Colors
        self.green = (5, 255, 0)
        self.blue = (0, 133, 255)
        self.red = (255, 0, 0)

        self.radius = 30

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
            # erase the circle in the middle
            pygame.draw.circle(self.window, (50, 50, 50), (self.width//2, self.height//2), self.width//8 + 10)
            # default state, no motion in the middle
            font = pygame.font.Font(None, 36)
            text = font.render("Hand Open", True, self.red)
            self.window.blit(text, (7*self.width//16, self.height//4))
            text = font.render("Hand Close", True, self.blue)
            self.window.blit(text, (7*self.width//16, 3*self.height//4))
            text = font.render("Wrist Extension", True, self.red)
            self.window.blit(text, (3*self.width//4, self.height//2))
            text = font.render("Wrist Flexion", True, self.blue)
            self.window.blit(text, (self.width//8, self.height//2))
            if input_class == 0:
                # self.publisher.publish("hand_close")
                # color the first circle
                pygame.draw.circle(self.window, self.green, (self.width//2, 5*self.height//8 - self.radius), self.radius)
            elif input_class == 1:
                # self.publisher.publish("hand_open")
                # color the second circle
                pygame.draw.circle(self.window, self.green, (self.width//2, 3*self.height//8 + self.radius), self.radius)
            elif input_class == 2:
                # self.publisher.publish("no_motion")
                # color the third circle
                pygame.draw.circle(self.window, self.green, (self.width//2, self.height//2), self.radius)
            elif input_class == 3:
                # self.publisher.publish("wrist_extension")
                # color the fourth circle
                pygame.draw.circle(self.window, self.green, (5*self.width//8 - self.radius, self.height//2), self.radius)
            elif input_class == 4:
                # self.publisher.publish("wrist_flexion")
                # color the fifth circle
                pygame.draw.circle(self.window, self.green, (3*self.width//8 + self.radius, self.height//2), self.radius)
            else:
                return

    def stop_running_game(self):
        self.running = False
        rospy.signal_shutdown("Game Ended")

    def run_game(self):
        signal.signal(signal.SIGINT, self.stop_running_game)
        while self.running: 
            self.handle_emg()

            pygame.display.update()
            self.clock.tick(30)

        pygame.quit()