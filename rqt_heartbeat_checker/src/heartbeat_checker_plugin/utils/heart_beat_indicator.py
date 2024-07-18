import rospy

from std_msgs.msg import Header
from python_qt_binding.QtWidgets import (
    QLabel,
)
from .widget_set import COLOR

class HeartBeatIndicator(QLabel):
    
    def __init__(self, text, topic):
        super(HeartBeatIndicator, self).__init__(text)
        self.setObjectName(topic)        
        self.setText(text)
        self.style_red = COLOR["red"]
        self.style_green = COLOR["green"]
        self.max_count = 40
        self.count = 0
        self.sub = rospy.Subscriber(topic, Header, self.on_heart_beat, queue_size=1)

    def __del__(self):
        self.sub.unregister()

    def on_heart_beat(self, msg):
        self.count = 0

    def on_timer(self):
        if self.count < self.max_count:
            self.setStyleSheet(self.style_green)
        else:
            self.setStyleSheet(self.style_red)
        self.count += 1

        if self.count % 30 == 0:
            rospy.logwarn(f"{self.text()} is not responding for {self.count} ms.")
