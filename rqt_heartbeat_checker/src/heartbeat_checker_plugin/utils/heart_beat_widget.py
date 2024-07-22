import rospy
import roslib

from .topic_list_indicator import TopicListIndicator

from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QGroupBox
from python_qt_binding.QtCore import QBasicTimer


class HeartBeatWidget(QWidget):
    def __init__(self):
        super(HeartBeatWidget, self).__init__()
        # Give QObjects reasonable names
        self.setWindowTitle("HeartBeatWidget")
        # Process standalone plugin command-line arguments

        self.init_ui()

    def init_ui(self):
        # create a vertical layout
        self.topic_indicator = TopicListIndicator("")

        # labels to show the status of each node
        self.heat_beat_group = QGroupBox("Heart Beat")
        self.heart_beat_group_layout = QVBoxLayout()

        self.heat_beat_group.setLayout(self.heart_beat_group_layout)

        self.vbox = QVBoxLayout()
        self.vbox.addLayout(self.topic_indicator)
        self.vbox.addWidget(self.heat_beat_group)
        self.setLayout(self.vbox)

        self.timer = QBasicTimer()
        self.timer.start(100, self)

    def update_ui(self):

        if self.topic_indicator.is_changed:
            self.topic_indicator.update_removable_topics()
            for topic_name in self.topic_indicator.label_list:
                print(self.topic_indicator.label_list[topic_name])
                self.heart_beat_group_layout.addWidget(self.topic_indicator.label_list[topic_name])

    def timerEvent(self, event):
        self.update_ui()
        for topic_name in self.topic_indicator.label_list:
            self.topic_indicator.label_list[topic_name].on_timer()

    def shutdown_plugin(self):
        for topic_name in self.topic_indicator.topic_list:
            del self.topic_indicator.label_list[topic_name]
        self.timer.stop()
