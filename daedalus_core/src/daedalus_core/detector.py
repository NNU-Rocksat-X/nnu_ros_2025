#! /usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy
from daedalus_msgs.msg import inhibit_detection

POLL_FREQ = 10.0 # freq in hz
INHIBIT_PIN_0 = 11
INHIBIT_PIN_1 = 13


GPIO.setmode(GPIO.BCM)

class Detector:
    def __init__(self, pin_number, persistence_threshold):
        self.pin_number = pin_number
        self.persistence_threshold = persistence_threshold

        self.persistence = 0
        self.state = False

        GPIO.setup(pin_number, GPIO.IN)


    def detect(self):
        current_state = GPIO.input(self.pin_number)

        if (current_state):
            self.persistence += 1
        else:
            self.persistence = 0

        if self.persistence > self.persistence_threshold:
            self.state = True
        else:
            self.state = False

        return self.state

    def get_state(self):
        return self.state

    def cleanup(self):
        GPIO.cleanup()


if __name__ == "__main__":
    p1 = Detector(pin_number=5, persistence_threshold=20)

    pub = rospy.Publisher("inhibit_detection", inhibit_detection, queue_size=1)

    # Poll the gpio at 10 hz
    while not rospy.is_shutdown():
        time.sleep(1/POLL_FREQ)
        p1.detect()

        print(p1.get_state())
    
    p1.cleanup()
